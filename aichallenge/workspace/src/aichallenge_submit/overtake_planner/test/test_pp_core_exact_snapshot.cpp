#include "overtake_planner/pp_core_exact_snapshot.hpp"

#include <gtest/gtest.h>

namespace {

using multi_purpose_mpc_ros_msgs::msg::ControllerExecutionEnvelope;
using overtake_planner::PurePursuitCandidateBinding;

std::pair<ControllerExecutionEnvelope, PurePursuitCandidateBinding>
makeCompleteSnapshot() {
  ControllerExecutionEnvelope envelope;
  envelope.header.frame_id = "map";
  envelope.header.stamp.sec = 12;
  envelope.schema_version = 2U;
  envelope.producer_instance_id = 99U;
  envelope.command_sequence = 7U;
  envelope.plan_generation = 42U;
  auto &command = envelope.command_envelope;
  command.header = envelope.header;
  command.schema_version = 2U;
  command.producer_instance_id = envelope.producer_instance_id;
  command.command_sequence = envelope.command_sequence;
  command.plan_generation = envelope.plan_generation;
  command.command.stamp = envelope.header.stamp;
  command.command.lateral.stamp = envelope.header.stamp;
  command.command.longitudinal.stamp = envelope.header.stamp;
  command.command.lateral.steering_tire_angle = 0.01F;
  command.command.lateral.steering_tire_rotation_rate = 0.0F;
  command.plan_sample_key.race_arm_epoch = 5U;
  command.plan_sample_key.planner_instance_id = 77U;
  command.plan_sample_key.attempt_id = 8U;
  command.plan_sample_key.target_vehicle_id = "D2";
  command.plan_sample_key.pass_direction = -1;
  command.plan_sample_key.connector_transaction_id = 9U;
  command.plan_sample_key.plan_stamp = envelope.header.stamp;
  command.plan_sample_key.plan_generation = envelope.plan_generation;
  command.candidate_revision = 3U;
  command.candidate_content_sha256.fill(0x44U);

  auto &witness = envelope.witness;
  witness.schema_version = 2U;
  witness.diagnostic_only = true;
  witness.exact_snapshot_complete = true;
  witness.exact_snapshot_reason = "complete";
  witness.authority_eligible = false;
  witness.plan_sample_key = command.plan_sample_key;
  witness.candidate_revision = command.candidate_revision;
  witness.candidate_content_sha256 = command.candidate_content_sha256;
  witness.base_source_stamp = envelope.header.stamp;
  witness.base_source_generation = 4U;
  witness.base_source_point_count = 130U;
  witness.base_source_sha256.fill(0x11U);
  witness.controller_implementation_sha256.fill(0x22U);
  witness.controller_config_sha256.fill(0x33U);
  witness.control_pose_stamp = envelope.header.stamp;
  witness.diagnostic_lease_duration_sec = 0.05F;
  witness.exact_geometry_original_point_count = 130U;
  witness.exact_geometry_first_source_index = 120U;
  witness.exact_geometry_last_source_index = 125U;
  witness.nearest_trajectory_index = 120U;
  witness.selected_lookahead_trajectory_index = 124U;
  witness.source_stamp = envelope.header.stamp;
  witness.applied_trajectory.header = envelope.header;
  for (std::size_t index = 120U; index <= 125U; ++index) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(index) * 0.1;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 1.0F;
    witness.applied_trajectory.points.push_back(point);
  }
  witness.control_pose.position.x = 12.5;
  witness.control_pose.orientation.w = 1.0;
  witness.control_speed_mps = 1.0;
  witness.resolved_lookahead_distance_m = 0.4;
  witness.geometric_steering_tire_angle_rad = 0.0;
  witness.curvature_feedforward_steering_rad = 0.01;
  witness.raw_exact_steering_tire_angle_rad = 0.01;
  witness.steering_output_gain = 1.0;
  witness.requested_output_steering_tire_angle_rad = 0.01;
  witness.limiter_reference_valid = true;
  witness.limiter_reference_steering_rad = 0.0;
  witness.command_dt_sec = 0.01;
  witness.requested_steering_tire_rotation_rate_radps = 1.0;
  witness.bounded_exact_steering_tire_angle_rad = 0.01;
  witness.bounded_exact_steering_tire_rotation_rate_radps = 1.0;
  witness.wheelbase_m = 1.087;
  witness.hard_steering_angle_limit_rad = 0.64;
  witness.hard_steering_rate_limit_radps = 128.0;

  PurePursuitCandidateBinding expected;
  expected.race_arm_epoch = 5U;
  expected.planner_instance_id = 77U;
  expected.attempt_id = 8U;
  expected.target_vehicle_id = "D2";
  expected.pass_direction = -1;
  expected.connector_transaction_id = 9U;
  expected.plan_stamp_sec = 12.0;
  expected.plan_generation = 42U;
  expected.candidate_revision = 3U;
  expected.candidate_content_sha256.fill(0x44U);
  return {envelope, expected};
}

TEST(PurePursuitExactSnapshot, AcceptsExactSlidingWindowAndScalars) {
  auto [envelope, expected] = makeCompleteSnapshot();
  const auto result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.02);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.reason, "complete");
  EXPECT_EQ(result.evaluator_input.nearest_source_index, 120U);
  EXPECT_EQ(result.selected_lookahead_source_index, 124U);
  EXPECT_NEAR(result.valid_until_sec, 12.05, 1.0e-6);
}

TEST(PurePursuitExactSnapshot,
     GoldenEvaluatorParityMatchesIndependentControllerScalars) {
  auto [envelope, expected] = makeCompleteSnapshot();
  const auto snapshot = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.02);
  ASSERT_TRUE(snapshot.valid) << snapshot.reason;

  overtake_planner::FrenetFrame frame;
  frame.setReference({
      {0.0, 0.0, 0.0, 0.0, 0.0, 5.0},
      {100.0, 100.0, 0.0, 0.0, 0.0, 5.0},
      {200.0, 100.0, 100.0, 1.5707963267948966, 0.0, 5.0},
  });
  overtake_planner::CandidateTrajectory candidate;
  for (std::size_t index = 0U; index < 130U; ++index) {
    const double offset_m = static_cast<double>(index) * 0.1;
    candidate.x.push_back(offset_m);
    candidate.y.push_back(0.0);
    candidate.yaw.push_back(0.0);
    candidate.t.push_back(offset_m);
    candidate.longitudinal_offsets_m.push_back(offset_m);
    candidate.predicted_speed_mps.push_back(1.0);
    candidate.v_ref.push_back(1.0);
  }
  overtake_planner::CartesianTrackabilityConfig config;
  config.wheelbase_m = 1.087;
  config.steering_gain = 1.0;
  config.max_steering_angle_rad = 0.64;
  config.max_steering_rate_radps = 128.0;
  config.max_yaw_tangent_error_rad = 0.1;
  config.pure_pursuit_required_arc_m = 0.4;
  config.target_d_m = 0.0;
  config.target_d_deadline_arc_m = 1.0;

  const auto result =
      overtake_planner::CartesianTrackabilityEvaluator(frame).evaluate(
          candidate, snapshot.evaluator_input, config);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.pure_pursuit_target_source_index,
            snapshot.selected_lookahead_source_index);
  EXPECT_NEAR(result.pure_pursuit_lookahead_distance_m,
              snapshot.evaluator_input.active_lookahead_distance_m, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_geometric_steering_angle_rad,
              snapshot.geometric_steering_tire_angle_rad, 1.0e-12);
  EXPECT_NEAR(result.curvature_feedforward_steering_angle_rad,
              envelope.witness.curvature_feedforward_steering_rad, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_raw_steering_angle_rad,
              snapshot.raw_steering_tire_angle_rad, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_requested_steering_angle_rad,
              snapshot.requested_output_steering_tire_angle_rad, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_bounded_steering_angle_rad,
              snapshot.bounded_steering_tire_angle_rad, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_requested_steering_rate_radps,
              snapshot.requested_steering_rate_radps, 1.0e-12);
}

TEST(PurePursuitExactSnapshot, RejectsCandidateMutationIndependently) {
  auto [envelope, expected] = makeCompleteSnapshot();
  envelope.witness.candidate_content_sha256[0] ^= 0xffU;
  const auto result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.02);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "current_candidate_binding_mismatch");
}

TEST(PurePursuitExactSnapshot, RejectsSplitCommandTimestampMutation) {
  auto [envelope, expected] = makeCompleteSnapshot();
  envelope.command_envelope.command.lateral.stamp.nanosec = 1U;
  const auto result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.02);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "atomic_command_binding_invalid");
}

TEST(PurePursuitExactSnapshot, RejectsTruncatedPrefixMasqueradingAsWindow) {
  auto [envelope, expected] = makeCompleteSnapshot();
  envelope.witness.exact_geometry_first_source_index = 0U;
  const auto result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.02);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "bounded_geometry_invalid");
}

TEST(PurePursuitExactSnapshot, RejectsZeroFutureAndExpiredCommandStamps) {
  auto [envelope, expected] = makeCompleteSnapshot();
  auto result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.20);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "diagnostic_lease_expired");

  result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 11.95, 0.05);
  EXPECT_TRUE(result.valid) << result.reason;

  result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 11.95 - 1.0e-6, 0.05);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "command_stamp_future_or_invalid");

  const builtin_interfaces::msg::Time zero_stamp;
  envelope.header.stamp = zero_stamp;
  envelope.command_envelope.header.stamp = zero_stamp;
  envelope.command_envelope.command.stamp = zero_stamp;
  envelope.command_envelope.command.lateral.stamp = zero_stamp;
  envelope.command_envelope.command.longitudinal.stamp = zero_stamp;
  envelope.command_envelope.plan_sample_key.plan_stamp = zero_stamp;
  envelope.witness.control_pose_stamp = zero_stamp;
  envelope.witness.plan_sample_key.plan_stamp = zero_stamp;
  expected.plan_stamp_sec = 0.0;
  result = overtake_planner::validatePurePursuitExactSnapshotV2(envelope,
                                                                expected, 0.0);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "current_candidate_binding_mismatch");
}

TEST(PurePursuitExactSnapshot,
     RejectsWireRateAndIndependentlyRecomputesLimiterWitness) {
  auto [envelope, expected] = makeCompleteSnapshot();

  envelope.command_envelope.command.lateral.steering_tire_rotation_rate = 1.0F;
  auto result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.02);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "command_wire_contract_invalid");

  envelope.command_envelope.command.lateral.steering_tire_rotation_rate = 0.0F;
  envelope.witness.bounded_exact_steering_tire_rotation_rate_radps += 0.01;
  result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.02);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "exact_scalar_relation_invalid");

  envelope.witness.bounded_exact_steering_tire_rotation_rate_radps = 1.0;
  envelope.witness.steering_rate_limited = true;
  result = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.02);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "exact_scalar_relation_invalid");
}

TEST(PurePursuitExactSnapshot, CanonicalDigestDetectsSameTupleMutation) {
  auto [first_envelope, expected] = makeCompleteSnapshot();
  auto second_envelope = first_envelope;
  second_envelope.witness.applied_trajectory.points[2].pose.position.y += 0.01;
  const auto first = overtake_planner::validatePurePursuitExactSnapshotV2(
      first_envelope, expected, 12.02);
  const auto second = overtake_planner::validatePurePursuitExactSnapshotV2(
      second_envelope, expected, 12.02);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_NE(first.bounded_geometry_sha256, second.bounded_geometry_sha256);
}

} // namespace
