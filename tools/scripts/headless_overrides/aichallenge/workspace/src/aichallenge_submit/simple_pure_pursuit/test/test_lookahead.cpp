#include "simple_pure_pursuit/delay_compensation.hpp"
#include "simple_pure_pursuit/lookahead.hpp"
#include "simple_pure_pursuit/overtake_override_contract.hpp"
#include "simple_pure_pursuit/safety.hpp"
#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <gtest/gtest.h>
#include <multi_purpose_mpc_ros_msgs/msg/controller_applied_transport_status.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/controller_command_envelope.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/controller_execution_envelope.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/overtake_plan.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <limits>
#include <optional>

namespace {

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

static_assert(
    rosidl_generator_traits::has_bounded_size<
        multi_purpose_mpc_ros_msgs::msg::ControllerAppliedTransportStatus>::
        value);

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw_rad) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);

  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

TrajectoryPoint makePoint(double x, double y, double yaw_rad) {
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = yawToQuaternion(yaw_rad);
  return point;
}

TrajectoryPoint makePointWithSpeed(double x, double y, double yaw_rad,
                                   double speed_mps) {
  auto point = makePoint(x, y, yaw_rad);
  point.longitudinal_velocity_mps = speed_mps;
  return point;
}

Trajectory makeStraightThenArcTrajectory() {
  constexpr double radius_m = 10.0;
  constexpr double step_rad = 0.10;
  Trajectory trajectory;
  for (int i = 0; i <= 5; ++i) {
    trajectory.points.push_back(makePoint(static_cast<double>(i), 0.0, 0.0));
  }
  for (int i = 1; i <= 8; ++i) {
    const double theta = step_rad * static_cast<double>(i);
    trajectory.points.push_back(makePoint(5.0 + radius_m * std::sin(theta),
                                          radius_m * (1.0 - std::cos(theta)),
                                          theta));
  }
  return trajectory;
}

Trajectory makeArcTrajectory(double direction) {
  constexpr double radius_m = 10.0;
  constexpr double step_rad = 0.10;
  Trajectory trajectory;
  for (int i = 0; i < 8; ++i) {
    const double theta = step_rad * static_cast<double>(i);
    trajectory.points.push_back(makePoint(
        radius_m * std::sin(theta),
        direction * radius_m * (1.0 - std::cos(theta)), direction * theta));
  }
  return trajectory;
}

multi_purpose_mpc_ros_msgs::msg::ControllerCommandEnvelope
makeExecutionTestCommandEnvelope() {
  autoware_auto_control_msgs::msg::AckermannControlCommand command;
  command.stamp.sec = 12;
  command.longitudinal.stamp = command.stamp;
  command.longitudinal.speed = 1.0;
  command.longitudinal.acceleration = 0.2;
  command.lateral.stamp = command.stamp;
  command.lateral.steering_tire_angle = 0.1;
  command.lateral.steering_tire_rotation_rate = 0.2;

  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus status;
  status.header.stamp = command.stamp;
  status.header.frame_id = "base_link";
  status.plan_generation = 42U;
  return simple_pure_pursuit::makeControllerCommandEnvelopeV1(command, status,
                                                              99U, 7U);
}

std_msgs::msg::Float32MultiArray makeExecutionTestSourcePayload() {
  std_msgs::msg::Float32MultiArray source;
  source.layout.data_offset = 1U;
  source.layout.dim.resize(1U);
  source.layout.dim.front().label = "override_wire";
  source.layout.dim.front().size = 3U;
  source.layout.dim.front().stride = 3U;
  source.data = {1.0F, 2.0F, 3.0F};
  return source;
}

simple_pure_pursuit::ControllerExecutionWitnessInput
makeCompleteExecutionWitnessInput(
    const multi_purpose_mpc_ros_msgs::msg::ControllerCommandEnvelope
        &command_envelope) {
  simple_pure_pursuit::ControllerExecutionWitnessInput input;
  input.header = command_envelope.header;
  input.controller_role =
      multi_purpose_mpc_ros_msgs::msg::ControllerExecutionWitness::ROLE_PRIMARY;
  input.trajectory_source = multi_purpose_mpc_ros_msgs::msg::
      ControllerExecutionWitness::SOURCE_REFERENCE_OVERRIDE;
  input.source_generation = command_envelope.plan_generation;
  input.source_payload = simple_pure_pursuit::canonicalizeSourcePayload(
      makeExecutionTestSourcePayload());
  input.reference_stamp = command_envelope.header.stamp;
  input.source_stamp = command_envelope.header.stamp;
  input.base_trajectory.header = command_envelope.header;
  input.base_trajectory.points = {makePointWithSpeed(0.0, 0.0, 0.0, 1.0),
                                  makePointWithSpeed(1.0, 0.0, 0.0, 1.0)};
  input.applied_trajectory = input.base_trajectory;
  input.applied_trajectory.points[1].pose.position.y = 0.2;
  input.nearest_trajectory_index = 0U;
  input.control_pose = input.applied_trajectory.points.front().pose;
  input.trajectory_progress_m = 0.0;
  input.available_spatial_horizon_m = 1.0;
  input.required_spatial_horizon_m = 0.5;
  input.raw_steering_tire_angle_rad = 0.12;
  input.bounded_steering_tire_angle_rad = 0.1;
  input.raw_steering_tire_rotation_rate_radps = 0.3;
  input.bounded_steering_tire_rotation_rate_radps = 0.2;
  input.steering_tire_angle_limit_rad = 0.35;
  input.steering_tire_rotation_rate_limit_radps = 3.0;
  input.rollout_samples.resize(2U);
  input.rollout_samples[0].elapsed_time_sec = 0.0F;
  input.rollout_samples[0].progress_m = 0.0F;
  input.rollout_samples[0].pose = input.control_pose;
  input.rollout_samples[0].speed_mps = 1.0F;
  input.rollout_samples[0].raw_steering_tire_angle_rad = 0.12F;
  input.rollout_samples[0].bounded_steering_tire_angle_rad = 0.1F;
  input.rollout_samples[0].raw_steering_tire_rotation_rate_radps = 0.3F;
  input.rollout_samples[0].bounded_steering_tire_rotation_rate_radps = 0.2F;
  input.rollout_samples[1] = input.rollout_samples[0];
  input.rollout_samples[1].elapsed_time_sec = 0.1F;
  input.rollout_samples[1].progress_m = 1.0F;
  input.rollout_samples[1].pose = input.applied_trajectory.points[1].pose;
  return input;
}

multi_purpose_mpc_ros_msgs::msg::OvertakePlan makeExecutionTestPlan(
    const multi_purpose_mpc_ros_msgs::msg::ControllerCommandEnvelope
        &command_envelope,
    const simple_pure_pursuit::ControllerExecutionWitnessInput &input) {
  multi_purpose_mpc_ros_msgs::msg::OvertakePlan plan;
  plan.header = command_envelope.header;
  plan.plan_generation = command_envelope.plan_generation;
  plan.attempt_id = 8U;
  plan.target_vehicle_id = "D2";
  plan.pass_direction = -1;
  plan.trajectory_authorized = true;
  plan.trajectory = input.applied_trajectory;
  return plan;
}

simple_pure_pursuit::ControllerAppliedEnvelopeInput
makeCompleteControllerAppliedInput(
    const multi_purpose_mpc_ros_msgs::msg::ControllerCommandEnvelope
        &command_envelope) {
  simple_pure_pursuit::ControllerAppliedEnvelopeInput input;
  input.header = command_envelope.header;
  input.controller_role =
      multi_purpose_mpc_ros_msgs::msg::ControllerAppliedEnvelope::ROLE_PRIMARY;
  input.geometry_relation = multi_purpose_mpc_ros_msgs::msg::
      ControllerAppliedEnvelope::GEOMETRY_RELATION_DERIVED_REFERENCE_OVERRIDE;
  input.source_generation = command_envelope.plan_generation;
  input.source_wire = simple_pure_pursuit::canonicalizeAw2SourceWire(
      makeExecutionTestSourcePayload());
  input.base_trajectory.header = command_envelope.header;
  input.base_trajectory.points = {makePointWithSpeed(0.0, 0.0, 0.0, 1.0),
                                  makePointWithSpeed(1.0, 0.0, 0.0, 1.0)};
  input.applied_trajectory = input.base_trajectory;
  input.applied_trajectory.points[1].pose.position.y = 0.2;
  input.control_pose = input.applied_trajectory.points.front().pose;
  input.control_pose_stamp = command_envelope.header.stamp;
  input.nearest_trajectory_index = 0U;
  input.speed_cap_trajectory_index = 0U;
  input.curvature_last_read_trajectory_index = 1U;
  input.lookahead_selected_trajectory_index = 1U;
  input.required_horizon_end_trajectory_index = 1U;
  input.trajectory_progress_m = 0.0;
  input.controller_adapter_implementation_sha256.fill(0x11U);
  input.controller_adapter_config_sha256.fill(0x22U);
  input.raw_controller_command = command_envelope.command;
  input.raw_controller_command.lateral.steering_tire_angle = 0.12;
  input.raw_controller_command.lateral.steering_tire_rotation_rate = 0.3;
  input.raw_steering_tire_angle_rad = 0.12;
  input.output_steering_tire_angle_rad =
      command_envelope.command.lateral.steering_tire_angle;
  input.raw_steering_tire_rotation_rate_radps = 0.3;
  input.output_steering_tire_rotation_rate_radps = 0.2;
  input.hard_actuator_limits_present = true;
  input.hard_steering_tire_angle_limit_rad = 0.35;
  input.hard_steering_tire_rotation_rate_limit_radps = 3.0;
  input.available_spatial_horizon_m = 1.0;
  input.required_spatial_horizon_m = 0.5;
  input.rollout_state = multi_purpose_mpc_ros_msgs::msg::
      ControllerAppliedEnvelope::ROLLOUT_COMPLETE;
  input.rollout_samples =
      makeCompleteExecutionWitnessInput(command_envelope).rollout_samples;
  return input;
}

multi_purpose_mpc_ros_msgs::msg::OvertakePlan makeAw2ExecutionTestPlan(
    const multi_purpose_mpc_ros_msgs::msg::ControllerCommandEnvelope
        &command_envelope,
    const simple_pure_pursuit::ControllerAppliedEnvelopeInput &input) {
  multi_purpose_mpc_ros_msgs::msg::OvertakePlan plan;
  plan.header = command_envelope.header;
  plan.phase = plan.PASSING;
  plan.plan_generation = command_envelope.plan_generation;
  plan.attempt_id = 8U;
  plan.target_vehicle_id = "D2";
  plan.pass_direction = -1;
  plan.trajectory_authorized = true;
  plan.lateral_maneuver_required = true;
  plan.trajectory = input.applied_trajectory;
  plan.aw2_identity_schema_version = 1U;
  plan.planner_instance_id = 77U;
  plan.race_arm_epoch = 5U;
  plan.connector_transaction_id = 9U;
  plan.candidate_revision = 3U;
  plan.candidate_content_sha256.fill(0x33U);
  return plan;
}

} // namespace

TEST(ControllerCommandEnvelope, V1ShadowSampleBindsFullCommandAndProof) {
  autoware_auto_control_msgs::msg::AckermannControlCommand command;
  command.stamp.sec = 12;
  command.stamp.nanosec = 34;
  command.longitudinal.stamp = command.stamp;
  command.longitudinal.speed = 1.5;
  command.longitudinal.acceleration = -0.4;
  command.longitudinal.jerk = 0.0;
  command.lateral.stamp = command.stamp;
  command.lateral.steering_tire_angle = -0.2;
  command.lateral.steering_tire_rotation_rate = 0.0;

  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus status;
  status.header.stamp = command.stamp;
  status.header.frame_id = "base_link";
  status.plan_generation = 42U;
  status.mpc_horizon_usable = true;
  status.pp_command_fresh = true;
  status.trajectory_tracking_usable = true;
  status.command_age_sec = 0.0F;
  status.reason = "ready";

  const auto envelope = simple_pure_pursuit::makeControllerCommandEnvelopeV1(
      command, status, 99U, 1U);

  EXPECT_EQ(envelope.schema_version, 1U);
  EXPECT_EQ(envelope.producer_instance_id, 99U);
  EXPECT_EQ(envelope.command_sequence, 1U);
  EXPECT_EQ(envelope.plan_generation, 42U);
  EXPECT_EQ(envelope.header, status.header);
  EXPECT_EQ(envelope.command, command);
  EXPECT_TRUE(envelope.mpc_horizon_usable);
  EXPECT_TRUE(envelope.pp_command_fresh);
  EXPECT_TRUE(envelope.trajectory_tracking_usable);
  EXPECT_FLOAT_EQ(envelope.command_age_sec, 0.0F);
  EXPECT_EQ(envelope.reason, "ready");
  EXPECT_DOUBLE_EQ(envelope.command.longitudinal.jerk, 0.0);
  EXPECT_DOUBLE_EQ(envelope.command.lateral.steering_tire_rotation_rate, 0.0);
}

TEST(ControllerCommandEnvelope, V2BindsExactPlannerCandidateIdentity) {
  autoware_auto_control_msgs::msg::AckermannControlCommand command;
  command.stamp.sec = 12;
  command.stamp.nanosec = 34;
  command.longitudinal.stamp = command.stamp;
  command.longitudinal.speed = 1.5;
  command.longitudinal.acceleration = 0.1;
  command.lateral.stamp = command.stamp;
  command.lateral.steering_tire_angle = -0.2;

  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus status;
  status.header.stamp = command.stamp;
  status.header.frame_id = "base_link";
  status.plan_generation = 42U;
  status.pp_command_fresh = true;
  status.trajectory_tracking_usable = true;

  multi_purpose_mpc_ros_msgs::msg::OvertakePlan plan;
  plan.header.stamp.sec = 11;
  plan.header.stamp.nanosec = 22;
  plan.aw2_identity_schema_version = 1U;
  plan.plan_generation = 42U;
  plan.planner_instance_id = 101U;
  plan.race_arm_epoch = 3U;
  plan.attempt_id = 7U;
  plan.target_vehicle_id = "D2";
  plan.pass_direction = -1;
  plan.connector_transaction_id = 55U;
  plan.candidate_revision = 9U;
  plan.candidate_content_sha256.fill(0x5aU);

  const auto envelope = simple_pure_pursuit::makeControllerCommandEnvelopeV1(
      command, status, 99U, 2U, &plan);

  EXPECT_EQ(envelope.schema_version, 2U);
  EXPECT_EQ(envelope.plan_sample_key.race_arm_epoch, 3U);
  EXPECT_EQ(envelope.plan_sample_key.planner_instance_id, 101U);
  EXPECT_EQ(envelope.plan_sample_key.attempt_id, 7U);
  EXPECT_EQ(envelope.plan_sample_key.target_vehicle_id, "D2");
  EXPECT_EQ(envelope.plan_sample_key.pass_direction, -1);
  EXPECT_EQ(envelope.plan_sample_key.connector_transaction_id, 55U);
  EXPECT_EQ(envelope.plan_sample_key.plan_stamp, plan.header.stamp);
  EXPECT_EQ(envelope.plan_sample_key.plan_generation, 42U);
  EXPECT_EQ(envelope.candidate_revision, 9U);
  EXPECT_EQ(envelope.candidate_content_sha256,
            plan.candidate_content_sha256);
}

TEST(ControllerCommandEnvelope, V1ShadowStopKeepsSequenceAndCanonicalZeros) {
  autoware_auto_control_msgs::msg::AckermannControlCommand command;
  command.stamp.sec = 21;
  command.longitudinal.stamp = command.stamp;
  command.longitudinal.speed = 0.0;
  command.longitudinal.acceleration = 0.0;
  command.longitudinal.jerk = 0.0;
  command.lateral.stamp = command.stamp;
  command.lateral.steering_tire_angle = 0.0;
  command.lateral.steering_tire_rotation_rate = 0.0;

  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus status;
  status.header.stamp = command.stamp;
  status.header.frame_id = "base_link";
  status.plan_generation = 0U;
  status.pp_command_fresh = true;
  status.trajectory_tracking_usable = false;
  status.command_age_sec = 0.0F;
  status.reason = "odometry_stale";

  const auto envelope = simple_pure_pursuit::makeControllerCommandEnvelopeV1(
      command, status, 99U, 2U);
  EXPECT_EQ(envelope.producer_instance_id, 99U);
  EXPECT_EQ(envelope.command_sequence, 2U);
  EXPECT_EQ(envelope.plan_generation, 0U);
  EXPECT_FALSE(envelope.trajectory_tracking_usable);
  EXPECT_DOUBLE_EQ(envelope.command.longitudinal.jerk, 0.0);
  EXPECT_DOUBLE_EQ(envelope.command.lateral.steering_tire_rotation_rate, 0.0);
}

TEST(ControllerExecutionEnvelope,
     V1BindsCommandTypedIdentityGeometryAndBoundedRollout) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteExecutionWitnessInput(command_envelope);
  const auto plan = makeExecutionTestPlan(command_envelope, input);

  const auto before = command_envelope;
  const auto envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);

  EXPECT_EQ(command_envelope, before);
  EXPECT_EQ(envelope.header, command_envelope.header);
  EXPECT_EQ(envelope.producer_instance_id,
            command_envelope.producer_instance_id);
  EXPECT_EQ(envelope.command_sequence, command_envelope.command_sequence);
  EXPECT_EQ(envelope.plan_generation, command_envelope.plan_generation);
  EXPECT_EQ(envelope.command_envelope, command_envelope);
  EXPECT_TRUE(envelope.witness.shadow_geometry_complete)
      << envelope.witness.incompleteness_reason;
  EXPECT_EQ(envelope.witness.incompleteness_reason, "complete");
  EXPECT_FALSE(envelope.witness.authority_eligible);
  EXPECT_EQ(envelope.witness.controller_role,
            multi_purpose_mpc_ros_msgs::msg::ControllerExecutionWitness::
                ROLE_PRIMARY);
  EXPECT_EQ(envelope.witness.identity_source,
            multi_purpose_mpc_ros_msgs::msg::ControllerExecutionWitness::
                IDENTITY_TYPED_OVERTAKE_PLAN);
  EXPECT_TRUE(envelope.witness.typed_plan_geometry_matches_applied);
  EXPECT_EQ(envelope.witness.attempt_id, 8U);
  EXPECT_EQ(envelope.witness.target_vehicle_id, "D2");
  EXPECT_EQ(envelope.witness.pass_direction, -1);
  EXPECT_EQ(envelope.witness.plan_stamp, plan.header.stamp);
  EXPECT_EQ(envelope.witness.applied_trajectory, input.applied_trajectory);
  EXPECT_EQ(envelope.witness.rollout_samples.size(), 2U);
  EXPECT_TRUE(envelope.witness.source_binding_complete);
  EXPECT_EQ(envelope.witness.source_generation, 42U);
  EXPECT_FALSE(envelope.witness.source_payload_fingerprint.empty());
  EXPECT_EQ(envelope.witness.source_payload_original_size_bytes,
            input.source_payload.original_size_bytes);

  input.controller_role = multi_purpose_mpc_ros_msgs::msg::
      ControllerExecutionWitness::ROLE_RECOVERY;
  const auto recovery = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_TRUE(recovery.witness.shadow_geometry_complete);
  EXPECT_EQ(recovery.witness.controller_role,
            multi_purpose_mpc_ros_msgs::msg::ControllerExecutionWitness::
                ROLE_RECOVERY);
  // AW1 is observation-only for every role. A future separately reviewed
  // PRIMARY consumer is the only domain that could ever gain authority.
  EXPECT_FALSE(recovery.witness.authority_eligible);
}

TEST(ControllerExecutionEnvelope,
     V1FailsClosedForMissingStaleOrMismatchedTypedIdentity) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  const auto input = makeCompleteExecutionWitnessInput(command_envelope);

  auto envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, nullptr, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.identity_source,
            multi_purpose_mpc_ros_msgs::msg::ControllerExecutionWitness::
                IDENTITY_NONE);
  EXPECT_EQ(envelope.witness.incompleteness_reason, "typed_plan_missing");

  auto plan = makeExecutionTestPlan(command_envelope, input);
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, false);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "typed_plan_stale_or_out_of_order");

  plan.plan_generation = 41U;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "typed_plan_generation_mismatch");
  EXPECT_EQ(envelope.witness.attempt_id, 0U);

  plan.plan_generation = 42U;
  plan.trajectory.points.front().pose.position.x = 1.0;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "typed_plan_geometry_relation_unproven");
  EXPECT_EQ(envelope.witness.identity_source,
            multi_purpose_mpc_ros_msgs::msg::ControllerExecutionWitness::
                IDENTITY_TYPED_OVERTAKE_PLAN);
  EXPECT_EQ(envelope.witness.attempt_id, 8U);
  EXPECT_FALSE(envelope.witness.typed_plan_geometry_matches_applied);

  plan.trajectory = input.applied_trajectory;
  plan.trajectory_authorized = false;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "typed_plan_trajectory_unauthorized");
  EXPECT_EQ(envelope.witness.identity_source,
            multi_purpose_mpc_ros_msgs::msg::ControllerExecutionWitness::
                IDENTITY_NONE);
}

TEST(ControllerExecutionEnvelope,
     V1RejectsUnknownNonfiniteAndOverLimitEvidenceWithoutGrowingArrays) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteExecutionWitnessInput(command_envelope);
  input.trajectory_source = multi_purpose_mpc_ros_msgs::msg::
      ControllerExecutionWitness::SOURCE_UNKNOWN;
  input.base_trajectory.points.resize(
      simple_pure_pursuit::kMaxExecutionTrajectoryPoints + 1U);
  input.applied_trajectory = input.base_trajectory;
  input.raw_steering_tire_angle_rad = std::numeric_limits<double>::infinity();
  input.rollout_samples.resize(
      simple_pure_pursuit::kMaxExecutionRolloutSamples + 1U);
  auto plan = makeExecutionTestPlan(command_envelope, input);

  auto envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "trajectory_source_unknown");
  EXPECT_LE(envelope.witness.base_trajectory.points.size(),
            simple_pure_pursuit::kMaxExecutionTrajectoryPoints);
  EXPECT_LE(envelope.witness.applied_trajectory.points.size(),
            simple_pure_pursuit::kMaxExecutionTrajectoryPoints);
  EXPECT_LE(envelope.witness.rollout_samples.size(),
            simple_pure_pursuit::kMaxExecutionRolloutSamples);

  input.trajectory_source = multi_purpose_mpc_ros_msgs::msg::
      ControllerExecutionWitness::SOURCE_REFERENCE_OVERRIDE;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "trajectory_point_limit_exceeded");

  input.base_trajectory.points.resize(2U);
  input.base_trajectory.points[0] = makePointWithSpeed(0.0, 0.0, 0.0, 1.0);
  input.base_trajectory.points[1] = makePointWithSpeed(1.0, 0.0, 0.0, 1.0);
  input.applied_trajectory = input.base_trajectory;
  input.control_pose = input.applied_trajectory.points.front().pose;
  plan.trajectory = input.applied_trajectory;
  input.raw_steering_tire_angle_rad = 0.36;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "steering_bounds_incomplete");

  input.raw_steering_tire_angle_rad = 0.0;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "rollout_sample_limit_exceeded");

  input.rollout_samples =
      makeCompleteExecutionWitnessInput(command_envelope).rollout_samples;
  input.source_stamp.nanosec = 1000000000U;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "source_reference_frame_or_stamp_invalid");

  input.source_stamp = command_envelope.header.stamp;
  input.raw_steering_tire_rotation_rate_radps =
      std::numeric_limits<double>::quiet_NaN();
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "steering_bounds_incomplete");

  input.raw_steering_tire_rotation_rate_radps = 3.1;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "steering_bounds_incomplete");
}

TEST(ControllerExecutionEnvelope,
     V1CanonicalSourceBindingCoversLayoutAndRejectsOverflow) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteExecutionWitnessInput(command_envelope);
  const auto plan = makeExecutionTestPlan(command_envelope, input);
  const auto first = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  ASSERT_TRUE(first.witness.source_binding_complete);

  auto changed_source = makeExecutionTestSourcePayload();
  changed_source.layout.dim.front().label = "different_layout";
  input.source_payload =
      simple_pure_pursuit::canonicalizeSourcePayload(changed_source);
  const auto changed = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  ASSERT_TRUE(changed.witness.source_binding_complete);
  EXPECT_NE(first.witness.source_payload_fingerprint,
            changed.witness.source_payload_fingerprint);

  std_msgs::msg::Float32MultiArray oversized_source;
  oversized_source.data.resize(
      simple_pure_pursuit::kMaxExecutionSourcePayloadBytes);
  input.source_payload =
      simple_pure_pursuit::canonicalizeSourcePayload(oversized_source);
  EXPECT_FALSE(input.source_payload.complete);
  EXPECT_EQ(input.source_payload.bytes.size(),
            simple_pure_pursuit::kMaxExecutionSourcePayloadBytes);
  EXPECT_GT(input.source_payload.original_size_bytes,
            simple_pure_pursuit::kMaxExecutionSourcePayloadBytes);
  const auto oversized = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(oversized.witness.shadow_geometry_complete);
  EXPECT_FALSE(oversized.witness.source_binding_complete);
  EXPECT_TRUE(oversized.witness.source_payload_fingerprint.empty());
  EXPECT_EQ(oversized.witness.incompleteness_reason,
            "source_binding_incomplete");
}

TEST(ControllerExecutionEnvelope,
     V1RejectsFrameMismatchAndIncompleteRolloutGeometry) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteExecutionWitnessInput(command_envelope);
  auto plan = makeExecutionTestPlan(command_envelope, input);

  input.base_trajectory.header.frame_id = "map";
  auto envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "source_reference_frame_or_stamp_invalid");

  input = makeCompleteExecutionWitnessInput(command_envelope);
  plan = makeExecutionTestPlan(command_envelope, input);
  plan.header.frame_id = "map";
  plan.trajectory.header = plan.header;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "typed_plan_header_or_frame_invalid");

  input = makeCompleteExecutionWitnessInput(command_envelope);
  plan = makeExecutionTestPlan(command_envelope, input);
  input.rollout_samples[1].elapsed_time_sec = -0.1F;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "rollout_geometry_incomplete");

  input = makeCompleteExecutionWitnessInput(command_envelope);
  input.rollout_samples.front().pose.position.x = 0.1;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "rollout_geometry_incomplete");

  input = makeCompleteExecutionWitnessInput(command_envelope);
  input.rollout_samples.back().progress_m = 0.25F;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "rollout_geometry_incomplete");

  input = makeCompleteExecutionWitnessInput(command_envelope);
  input.rollout_samples[1].progress_m = -0.1F;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason,
            "rollout_geometry_incomplete");

  input = makeCompleteExecutionWitnessInput(command_envelope);
  input.required_spatial_horizon_m = 0.0;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV1(
      command_envelope, input, &plan, true);
  EXPECT_FALSE(envelope.witness.shadow_geometry_complete);
  EXPECT_EQ(envelope.witness.incompleteness_reason, "spatial_horizon_invalid");
}

TEST(ControllerExecutionEnvelope,
     TypedPlanOrderingRejectsFutureAndOlderEvidence) {
  builtin_interfaces::msg::Time now;
  now.sec = 20;
  builtin_interfaces::msg::Time previous;
  previous.sec = 12;
  previous.nanosec = 200U;

  auto candidate = previous;
  candidate.sec = 21;
  EXPECT_FALSE(simple_pure_pursuit::typedPlanOrderAcceptable(
      candidate, 43U, now, true, previous, 42U));

  candidate.sec = 11;
  EXPECT_FALSE(simple_pure_pursuit::typedPlanOrderAcceptable(
      candidate, 41U, now, true, previous, 42U));

  candidate = previous;
  candidate.nanosec = 100U;
  EXPECT_FALSE(simple_pure_pursuit::typedPlanOrderAcceptable(
      candidate, 42U, now, true, previous, 42U));
  EXPECT_FALSE(simple_pure_pursuit::typedPlanOrderAcceptable(
      candidate, 43U, now, true, previous, 42U));

  candidate.sec = 13;
  candidate.nanosec = 0U;
  EXPECT_TRUE(simple_pure_pursuit::typedPlanOrderAcceptable(
      candidate, 42U, now, true, previous, 42U));
  EXPECT_TRUE(simple_pure_pursuit::typedPlanOrderAcceptable(
      candidate, 43U, now, true, previous, 42U));
}

TEST(ControllerAppliedEnvelope,
     V1PrimaryExactKeyProducesBoundedAtomicShadowRecord) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  const auto input = makeCompleteControllerAppliedInput(command_envelope);
  const auto plan = makeAw2ExecutionTestPlan(command_envelope, input);
  simple_pure_pursuit::ControllerAppliedBindingTracker tracker;

  const auto applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, &tracker);

  EXPECT_EQ(applied.schema_version, applied.SCHEMA_V1);
  EXPECT_FALSE(applied.authority_eligible);
  EXPECT_EQ(applied.evidence_state, applied.EVIDENCE_COMPLETE)
      << applied.evidence_reason;
  EXPECT_EQ(applied.controller_sample_key.plan_sample_key.race_arm_epoch, 5U);
  EXPECT_EQ(applied.controller_sample_key.plan_sample_key.planner_instance_id,
            77U);
  EXPECT_EQ(applied.controller_sample_key.plan_sample_key.attempt_id, 8U);
  EXPECT_EQ(applied.controller_sample_key.plan_sample_key.target_vehicle_id,
            "D2");
  EXPECT_EQ(applied.controller_sample_key.plan_sample_key.pass_direction, -1);
  EXPECT_EQ(
      applied.controller_sample_key.plan_sample_key.connector_transaction_id,
      9U);
  EXPECT_EQ(applied.controller_sample_key.plan_sample_key.plan_stamp,
            plan.header.stamp);
  EXPECT_EQ(applied.controller_sample_key.plan_sample_key.plan_generation, 42U);
  EXPECT_EQ(applied.controller_sample_key.controller_instance_id, 99U);
  EXPECT_EQ(applied.controller_sample_key.controller_sequence, 7U);
  EXPECT_EQ(applied.raw_controller_command, input.raw_controller_command);
  EXPECT_EQ(applied.output_controller_command, command_envelope.command);
  EXPECT_EQ(applied.base_geometry.original_point_count, 2U);
  EXPECT_EQ(applied.applied_geometry.first_source_index, 0U);
  EXPECT_EQ(applied.applied_geometry.last_source_index, 1U);
  EXPECT_EQ(applied.applied_geometry.speed_cap_source_index, 0U);
  EXPECT_EQ(applied.applied_geometry.required_horizon_end_source_index, 1U);
  EXPECT_EQ(applied.applied_geometry.full_source_digest_state,
            applied.applied_geometry.FULL_SOURCE_DIGEST_INDETERMINATE);
  EXPECT_EQ(applied.base_geometry.points.size(), 2U);
  EXPECT_EQ(applied.applied_geometry.points.size(), 2U);
  EXPECT_EQ(applied.rollout_samples.size(), 2U);
  EXPECT_TRUE(applied.source_wire_complete);
  EXPECT_NE(applied.applied_geometry.geometry_sha256,
            simple_pure_pursuit::Aw2Sha256Digest{});
  EXPECT_NE(applied.controller_applied_envelope_sha256,
            simple_pure_pursuit::Aw2Sha256Digest{});
}

TEST(ControllerAppliedEnvelope,
     ControllerUsedIntervalCopiesOnlyExactContiguousWindowDeterministically) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteControllerAppliedInput(command_envelope);
  input.geometry_relation = multi_purpose_mpc_ros_msgs::msg::
      ControllerAppliedEnvelope::GEOMETRY_RELATION_DIRECT_APPLIED;
  input.base_trajectory.points.clear();
  input.base_trajectory.points.reserve(1000U);
  for (std::size_t index = 0U; index < 1000U; ++index) {
    input.base_trajectory.points.push_back(
        makePointWithSpeed(static_cast<double>(index), 0.0, 0.0, 1.0));
  }
  input.applied_trajectory = input.base_trajectory;
  input.nearest_trajectory_index = 400U;
  input.speed_cap_trajectory_index = 405U;
  input.curvature_last_read_trajectory_index = 430U;
  input.lookahead_selected_trajectory_index = 420U;
  input.required_horizon_end_trajectory_index = 439U;
  input.required_spatial_horizon_m = 39.0;
  input.available_spatial_horizon_m = 39.0;
  input.control_pose = input.applied_trajectory.points[400U].pose;
  input.rollout_samples.front().pose = input.control_pose;
  input.rollout_samples.front().progress_m = 0.0F;
  input.rollout_samples.back().pose =
      input.applied_trajectory.points[439U].pose;
  input.rollout_samples.back().progress_m = 39.0F;
  const auto plan = makeAw2ExecutionTestPlan(command_envelope, input);

  const auto first = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  const auto second = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);

  ASSERT_EQ(first.evidence_state, first.EVIDENCE_COMPLETE)
      << first.evidence_reason;
  EXPECT_EQ(first.applied_geometry.original_point_count, 1000U);
  EXPECT_EQ(first.applied_geometry.first_source_index, 400U);
  EXPECT_EQ(first.applied_geometry.last_source_index, 439U);
  ASSERT_EQ(first.applied_geometry.points.size(), 40U);
  EXPECT_DOUBLE_EQ(first.applied_geometry.points.front().position_x_m, 400.0);
  EXPECT_DOUBLE_EQ(first.applied_geometry.points.back().position_x_m, 439.0);
  EXPECT_DOUBLE_EQ(first.applied_geometry.total_arc_length_m, 39.0);
  EXPECT_EQ(first.applied_geometry.full_source_digest_state,
            first.applied_geometry.FULL_SOURCE_DIGEST_INDETERMINATE);
  EXPECT_EQ(first.applied_geometry.geometry_sha256,
            second.applied_geometry.geometry_sha256);

  input.required_horizon_end_trajectory_index = 500U;
  const auto over_limit = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_EQ(over_limit.evidence_state,
            over_limit.EVIDENCE_GEOMETRY_LIMIT_EXCEEDED);

  input.required_horizon_end_trajectory_index = 1000U;
  const auto unavailable = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_EQ(unavailable.evidence_state,
            unavailable.EVIDENCE_HORIZON_INSUFFICIENT);
}

TEST(ControllerAppliedEnvelope, Sha256KnownVectorAndSameKeyMutationFailClosed) {
  const std::vector<std::uint8_t> abc{'a', 'b', 'c'};
  const simple_pure_pursuit::Aw2Sha256Digest expected{
      0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40,
      0xde, 0x5d, 0xae, 0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17,
      0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad};
  EXPECT_EQ(simple_pure_pursuit::aw2Sha256(abc), expected);

  const auto command_envelope = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteControllerAppliedInput(command_envelope);
  const auto plan = makeAw2ExecutionTestPlan(command_envelope, input);
  simple_pure_pursuit::ControllerAppliedBindingTracker tracker;
  const auto first = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, &tracker);
  const auto duplicate = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, &tracker);
  EXPECT_EQ(duplicate.evidence_state, duplicate.EVIDENCE_COMPLETE);
  EXPECT_EQ(first.controller_applied_envelope_sha256,
            duplicate.controller_applied_envelope_sha256);

  input.raw_controller_command.longitudinal.acceleration = 0.3;
  const auto conflict = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, &tracker);
  EXPECT_EQ(conflict.evidence_state, conflict.EVIDENCE_DUPLICATE_CONFLICT);
  EXPECT_FALSE(conflict.authority_eligible);

  auto next_command = command_envelope;
  next_command.command_sequence = 8U;
  auto mutated_plan = plan;
  mutated_plan.candidate_revision = 4U;
  const auto mutation = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      next_command, makeCompleteControllerAppliedInput(next_command),
      &mutated_plan, true, &tracker);
  EXPECT_EQ(mutation.evidence_state,
            mutation.EVIDENCE_SAME_GENERATION_PAYLOAD_MUTATION);

  auto regressed_command = command_envelope;
  regressed_command.command_sequence = 6U;
  const auto regression = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      regressed_command, makeCompleteControllerAppliedInput(regressed_command),
      &plan, true, &tracker);
  EXPECT_EQ(regression.evidence_state, regression.EVIDENCE_DUPLICATE_CONFLICT);
}

TEST(ControllerAppliedEnvelope,
     SchemaIsBoundedAndSourceWireRejectsNonfiniteOrOversize) {
  EXPECT_TRUE(
      rosidl_generator_traits::has_bounded_size<
          multi_purpose_mpc_ros_msgs::msg::ControllerAppliedEnvelope>::value);

  auto source = makeExecutionTestSourcePayload();
  source.data.front() = std::numeric_limits<float>::quiet_NaN();
  const auto nonfinite = simple_pure_pursuit::canonicalizeAw2SourceWire(source);
  EXPECT_FALSE(nonfinite.complete);
  EXPECT_TRUE(nonfinite.bytes.empty());
  EXPECT_EQ(nonfinite.sha256, simple_pure_pursuit::Aw2Sha256Digest{});

  source = makeExecutionTestSourcePayload();
  source.data.resize(simple_pure_pursuit::kMaxExecutionSourcePayloadBytes);
  const auto oversized = simple_pure_pursuit::canonicalizeAw2SourceWire(source);
  EXPECT_FALSE(oversized.complete);
  EXPECT_TRUE(oversized.bytes.empty());
  EXPECT_GT(oversized.original_size_bytes,
            simple_pure_pursuit::kMaxExecutionSourcePayloadBytes);
}

TEST(ControllerAppliedEnvelope, V1RejectsMissingOrMismatchedPlanIdentity) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  const auto input = makeCompleteControllerAppliedInput(command_envelope);
  auto plan = makeAw2ExecutionTestPlan(command_envelope, input);

  auto applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, nullptr, true, nullptr);
  EXPECT_EQ(applied.schema_version, applied.SCHEMA_INVALID);
  EXPECT_EQ(applied.evidence_state, applied.EVIDENCE_MISSING_INPUT);

  applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, false, nullptr);
  EXPECT_EQ(applied.evidence_state, applied.EVIDENCE_STALE_INPUT);

  plan.race_arm_epoch = 0U;
  applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_EQ(applied.evidence_state, applied.EVIDENCE_KEY_MISMATCH);

  plan = makeAw2ExecutionTestPlan(command_envelope, input);
  plan.pass_direction = 0;
  applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_EQ(applied.evidence_state, applied.EVIDENCE_KEY_MISMATCH);

  plan = makeAw2ExecutionTestPlan(command_envelope, input);
  plan.trajectory_authorized = false;
  applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_NE(applied.evidence_state, applied.EVIDENCE_COMPLETE);
  EXPECT_FALSE(applied.authority_eligible);
}

TEST(ControllerAppliedEnvelope,
     V1RejectsNonfiniteOverLimitAndUnreachableEvidence) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteControllerAppliedInput(command_envelope);
  auto plan = makeAw2ExecutionTestPlan(command_envelope, input);

  input.applied_trajectory.points.front().pose.position.x =
      std::numeric_limits<double>::quiet_NaN();
  auto applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_EQ(applied.evidence_state, applied.EVIDENCE_GEOMETRY_INVALID);

  input = makeCompleteControllerAppliedInput(command_envelope);
  input.base_trajectory.points.resize(101U);
  plan = makeAw2ExecutionTestPlan(command_envelope, input);
  applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_NE(applied.evidence_state, applied.EVIDENCE_GEOMETRY_LIMIT_EXCEEDED);
  EXPECT_EQ(applied.base_geometry.original_point_count, 101U);
  EXPECT_EQ(applied.base_geometry.points.size(), 2U);

  input = makeCompleteControllerAppliedInput(command_envelope);
  plan = makeAw2ExecutionTestPlan(command_envelope, input);
  input.raw_steering_tire_angle_rad = 0.36;
  applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_NE(applied.evidence_state, applied.EVIDENCE_COMPLETE);

  input = makeCompleteControllerAppliedInput(command_envelope);
  input.rollout_samples.resize(101U);
  applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_NE(applied.evidence_state, applied.EVIDENCE_COMPLETE);
  EXPECT_LE(applied.rollout_samples.size(), 100U);
}

TEST(ControllerAppliedEnvelope,
     RecoveryAndPpOwnedMissingLimitsRemainShadowIncomplete) {
  const auto command_envelope = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteControllerAppliedInput(command_envelope);
  const auto plan = makeAw2ExecutionTestPlan(command_envelope, input);

  input.controller_role =
      multi_purpose_mpc_ros_msgs::msg::ControllerAppliedEnvelope::ROLE_RECOVERY;
  auto applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_EQ(applied.evidence_state, applied.EVIDENCE_KEY_MISMATCH);
  EXPECT_FALSE(applied.authority_eligible);

  input = makeCompleteControllerAppliedInput(command_envelope);
  input.hard_actuator_limits_present = false;
  input.hard_steering_tire_angle_limit_rad = 0.0;
  input.hard_steering_tire_rotation_rate_limit_radps = 0.0;
  input.rollout_state = multi_purpose_mpc_ros_msgs::msg::
      ControllerAppliedEnvelope::ROLLOUT_UNAVAILABLE;
  input.rollout_samples.clear();
  applied = simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command_envelope, input, &plan, true, nullptr);
  EXPECT_EQ(applied.evidence_state, applied.EVIDENCE_ACTUATOR_LIMIT_UNKNOWN);
  EXPECT_FALSE(applied.authority_eligible);
}

TEST(OvertakeOverrideContract, SpeedOnlyV2ParsesWithoutLateralOffsets) {
  const auto contract = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 11.0F, 0.0F, 2.0F, 42.0F, 0.5F});

  ASSERT_TRUE(contract.has_value());
  EXPECT_EQ(contract->kind,
            simple_pure_pursuit::OvertakeOverrideContractKind::SPEED_ONLY_V2);
  EXPECT_EQ(contract->mode_id, 11);
  EXPECT_EQ(contract->generation, 42U);
  EXPECT_TRUE(contract->lateral_offsets.empty());
  ASSERT_EQ(contract->speed_caps.size(), 1U);
  EXPECT_NEAR(contract->speed_caps.front(), 0.5, 1.0e-9);
}

TEST(OvertakeOverrideContract,
     SpeedOnlyTrackingProofRequiresAppliedMatchingFiniteCap) {
  EXPECT_TRUE(simple_pure_pursuit::speedOnlyTrackingContractApplied(
      true, false, true, false, 42U, 42U, 0.5, 0.5));

  EXPECT_FALSE(simple_pure_pursuit::speedOnlyTrackingContractApplied(
      false, false, true, false, 42U, 42U, 0.5, 0.5));
  EXPECT_FALSE(simple_pure_pursuit::speedOnlyTrackingContractApplied(
      true, true, true, false, 42U, 42U, 0.5, 0.5));
  EXPECT_FALSE(simple_pure_pursuit::speedOnlyTrackingContractApplied(
      true, false, true, true, 42U, 42U, 0.5, 0.5));
  EXPECT_FALSE(simple_pure_pursuit::speedOnlyTrackingContractApplied(
      true, false, true, false, 41U, 42U, 0.5, 0.5));
  EXPECT_FALSE(simple_pure_pursuit::speedOnlyTrackingContractApplied(
      true, false, true, false, 42U, 42U, 0.5, 0.6));
  EXPECT_FALSE(simple_pure_pursuit::speedOnlyTrackingContractApplied(
      true, false, true, false, 42U, 42U,
      std::numeric_limits<double>::infinity(), 0.5));
  EXPECT_FALSE(simple_pure_pursuit::speedOnlyTrackingContractApplied(
      true, false, true, false, 42U, 42U, 0.5,
      std::numeric_limits<double>::quiet_NaN()));
}

TEST(OvertakeOverrideContract, InvalidV2FailsClosed) {
  const std::vector<std::vector<float>> malformed = {
      {1.0F, 0.0F, 0.0F, 2.0F, 42.0F, 0.5F},
      {1.0F, 11.0F, 0.0F, 2.0F, 0.0F, 0.5F},
      {1.0F, 11.0F, 0.0F, 2.0F, 42.0F, 0.0F},
      {1.0F, 11.0F, 0.0F, 2.0F, 42.0F, std::numeric_limits<float>::infinity()},
      {1.0F, 11.0F, 1.0F, 2.0F, 42.0F, 0.5F},
  };
  for (const auto &payload : malformed) {
    EXPECT_FALSE(simple_pure_pursuit::parseOvertakeOverrideContract(payload)
                     .has_value());
  }
}

TEST(OvertakeOverrideContract, ExplicitInactiveV1ClearsInsteadOfSpeedOnly) {
  const auto contract = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 0.0F, 0.0F, 1.0F, 42.0F});

  ASSERT_TRUE(contract.has_value());
  EXPECT_EQ(contract->kind,
            simple_pure_pursuit::OvertakeOverrideContractKind::INACTIVE);
  EXPECT_EQ(contract->mode_id, 0);
  EXPECT_EQ(contract->generation, 42U);
  EXPECT_TRUE(contract->speed_caps.empty());

  const auto legacy =
      simple_pure_pursuit::parseOvertakeOverrideContract({1.0F, 0.0F, 0.0F});
  ASSERT_TRUE(legacy.has_value());
  EXPECT_EQ(legacy->generation, 0U);
}

TEST(OvertakeOverrideContract, InvalidExplicitInactiveV1FailsClosed) {
  const std::vector<std::vector<float>> malformed = {
      {1.0F, 0.0F, 0.0F, 1.0F, 0.0F},
      {1.0F, 0.0F, 0.0F, 1.0F, std::numeric_limits<float>::infinity()},
      {1.0F, 0.0F, 0.0F, 1.0F, 16777216.0F},
  };
  for (const auto &payload : malformed) {
    EXPECT_FALSE(simple_pure_pursuit::parseOvertakeOverrideContract(payload)
                     .has_value());
  }
}

TEST(OvertakeOverrideContract, V4ParsesSpatialAxisAndIntent) {
  const auto contract = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 7.0F, 3.0F, 0.8F, 0.6F, 0.2F, 1.0F, 1.0F, 1.0F, 0.0F, 0.4F, 1.2F,
       4.0F, 44.0F, 2.0F});

  ASSERT_TRUE(contract.has_value());
  EXPECT_EQ(contract->kind, simple_pure_pursuit::OvertakeOverrideContractKind::
                                SPATIAL_LATERAL_AND_SPEED_V4);
  EXPECT_EQ(contract->generation, 44U);
  EXPECT_TRUE(contract->solver_horizon_authorized);
  EXPECT_TRUE(contract->mandatory_lateral_avoidance);
  ASSERT_EQ(contract->longitudinal_offsets_m.size(), 3U);
  EXPECT_NEAR(contract->longitudinal_offsets_m[0], 0.0, 1.0e-9);
  EXPECT_NEAR(contract->longitudinal_offsets_m[1], 0.4, 1.0e-6);
  EXPECT_NEAR(contract->longitudinal_offsets_m[2], 1.2, 1.0e-6);
}

TEST(OvertakeOverrideContract, V4DistanceSamplingIsIndependentOfPointIndex) {
  const std::vector<double> distances_m = {0.0, 0.4, 1.2};
  const std::vector<double> lateral_offsets_m = {0.8, 0.6, 0.2};

  const auto at_quarter_meter =
      simple_pure_pursuit::sampleOvertakeProfileByDistance(
          distances_m, lateral_offsets_m, 0.2);
  const auto at_one_meter =
      simple_pure_pursuit::sampleOvertakeProfileByDistance(
          distances_m, lateral_offsets_m, 1.0);
  const auto beyond_horizon =
      simple_pure_pursuit::sampleOvertakeProfileByDistance(
          distances_m, lateral_offsets_m, 2.0);

  ASSERT_TRUE(at_quarter_meter.has_value());
  ASSERT_TRUE(at_one_meter.has_value());
  EXPECT_FALSE(beyond_horizon.has_value());
  EXPECT_NEAR(at_quarter_meter.value(), 0.7, 1.0e-9);
  EXPECT_NEAR(at_one_meter.value(), 0.3, 1.0e-9);
}

TEST(OvertakeOverrideContract,
     ProvenSpatialEndpointUsesTerminalLateralAndSpeedValues) {
  const std::vector<double> distances_m = {0.0, 2.0, 3.875};
  const std::vector<double> lateral_offsets_m = {0.0, 0.5, 1.25};
  const std::vector<double> speed_caps_mps = {1.3, 1.0, 0.75};
  constexpr std::size_t endpoint_index = 42U;
  const double representable_boundary = std::nextafter(
      3.875 + simple_pure_pursuit::kSpatialProfileEndpointToleranceM, 3.875);

  for (const double query_distance_m :
       {3.875, 3.875000000005368, representable_boundary}) {
    SCOPED_TRACE(query_distance_m);
    const auto normalized_distance =
        simple_pure_pursuit::normalizeProvenSpatialEndpointDistance(
            query_distance_m, endpoint_index, endpoint_index,
            distances_m.back());
    const auto lateral =
        simple_pure_pursuit::sampleOvertakeProfileAtProvenEndpoint(
            distances_m, lateral_offsets_m, query_distance_m, endpoint_index,
            endpoint_index);
    const auto speed =
        simple_pure_pursuit::sampleOvertakeProfileAtProvenEndpoint(
            distances_m, speed_caps_mps, query_distance_m, endpoint_index,
            endpoint_index);
    ASSERT_TRUE(normalized_distance.has_value());
    EXPECT_DOUBLE_EQ(normalized_distance.value(), distances_m.back());
    ASSERT_TRUE(lateral.has_value());
    ASSERT_TRUE(speed.has_value());
    EXPECT_DOUBLE_EQ(lateral.value(), lateral_offsets_m.back());
    EXPECT_DOUBLE_EQ(speed.value(), speed_caps_mps.back());
  }
}

TEST(OvertakeOverrideContract,
     ProvenSpatialEndpointRejectsDistanceOutsideAbsoluteTolerance) {
  const std::vector<double> distances_m = {0.0, 3.875};
  const std::vector<double> values = {0.0, 1.25};
  const double just_outside =
      distances_m.back() +
      simple_pure_pursuit::kSpatialProfileEndpointToleranceM + 1.0e-12;

  EXPECT_FALSE(simple_pure_pursuit::sampleOvertakeProfileAtProvenEndpoint(
                   distances_m, values, just_outside, 9U, 9U)
                   .has_value());
}

TEST(OvertakeOverrideContract,
     SpatialEndpointNormalizationRequiresExactPointProvenance) {
  const std::vector<double> distances_m = {0.0, 3.875};
  const std::vector<double> values = {0.0, 1.25};
  const double observed_overshoot = 3.875000000005368;

  EXPECT_FALSE(simple_pure_pursuit::sampleOvertakeProfileAtProvenEndpoint(
                   distances_m, values, observed_overshoot, 8U, 9U)
                   .has_value());
  EXPECT_FALSE(simple_pure_pursuit::sampleOvertakeProfileByDistance(
                   distances_m, values, observed_overshoot)
                   .has_value());
}

TEST(OvertakeOverrideContract, InvalidV4SpatialAxisFailsClosed) {
  const std::vector<std::vector<float>> malformed = {
      {1.0F, 7.0F, 2.0F, 0.8F, 0.2F, 1.0F, 1.0F, 0.1F, 1.0F, 4.0F, 44.0F, 2.0F},
      {1.0F, 7.0F, 2.0F, 0.8F, 0.2F, 1.0F, 1.0F, 0.0F, -0.1F, 4.0F, 44.0F,
       2.0F},
      {1.0F, 7.0F, 2.0F, 0.8F, 0.2F, 1.0F, 1.0F, 0.0F, 1.0F, 3.0F, 44.0F, 2.0F},
  };

  for (const auto &payload : malformed) {
    EXPECT_FALSE(simple_pure_pursuit::parseOvertakeOverrideContract(payload)
                     .has_value());
  }
}

TEST(OvertakeOverrideContract, SpatialHorizonRequiresTwoPointsAndLookahead) {
  EXPECT_FALSE(
      simple_pure_pursuit::spatialOverrideHorizonSufficient(1U, 0.0, 0.8));
  EXPECT_FALSE(
      simple_pure_pursuit::spatialOverrideHorizonSufficient(2U, 0.5, 0.8));
  EXPECT_TRUE(
      simple_pure_pursuit::spatialOverrideHorizonSufficient(3U, 0.8, 0.8));
}

TEST(OvertakeOverrideContract,
     LowSpeedSpatialHorizonUsesCurrentSpeedCoverageWithoutSelfLock) {
  const double normal_lookahead_m = 3.75;
  const double minimum_executable_arc_m =
      simple_pure_pursuit::minimumExecutableSpatialHorizonArc(
          normal_lookahead_m, 0.0, 0.50, 0.75, 0.25, 1.0, 0.20);

  EXPECT_NEAR(minimum_executable_arc_m, 0.50, 1.0e-9);
  EXPECT_TRUE(simple_pure_pursuit::spatialOverrideHorizonSufficient(
      2U, 0.570833333, minimum_executable_arc_m));
  EXPECT_FALSE(simple_pure_pursuit::spatialOverrideHorizonSufficient(
      2U, 0.49, minimum_executable_arc_m));
}

TEST(OvertakeOverrideContract,
     MovingSpatialHorizonRequiresConfiguredForwardTimeCoverage) {
  const double minimum_executable_arc_m =
      simple_pure_pursuit::minimumExecutableSpatialHorizonArc(
          8.0, 4.0, 0.50, 0.75, 0.25, 1.0, 0.20);

  EXPECT_NEAR(minimum_executable_arc_m, 8.98, 1.0e-9);
  EXPECT_FALSE(simple_pure_pursuit::spatialOverrideHorizonSufficient(
      10U, 8.97, minimum_executable_arc_m));
  EXPECT_TRUE(simple_pure_pursuit::spatialOverrideHorizonSufficient(
      10U, 8.98, minimum_executable_arc_m));
}

TEST(OvertakeOverrideContract,
     SpatialHorizonSafetyBoundsCannotBeRelaxedByConfiguration) {
  const double relaxed_configuration_arc_m =
      simple_pure_pursuit::minimumExecutableSpatialHorizonArc(
          3.75, 0.0, 0.01, 0.01, 0.01, 1.0, 2.0);
  const double invalid_deceleration_arc_m =
      simple_pure_pursuit::minimumExecutableSpatialHorizonArc(
          3.75, 1.0, 0.50, 0.75, 0.25, 1.01, 0.20);
  const double invalid_speed_arc_m =
      simple_pure_pursuit::minimumExecutableSpatialHorizonArc(
          3.75, std::numeric_limits<double>::quiet_NaN(), 0.50, 0.75, 0.25, 1.0,
          0.20);

  EXPECT_NEAR(relaxed_configuration_arc_m,
              simple_pure_pursuit::kMinimumSpatialHorizonArcM, 1.0e-9);
  EXPECT_TRUE(std::isinf(invalid_deceleration_arc_m));
  EXPECT_TRUE(std::isinf(invalid_speed_arc_m));
}

TEST(OvertakeOverrideContract,
     SpeedOnlyV2KeepsBaselineLateralAndRequestsImmediateBraking) {
  const auto contract = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 11.0F, 0.0F, 2.0F, 42.0F, 0.5F});

  ASSERT_TRUE(contract.has_value());
  ASSERT_EQ(contract->kind,
            simple_pure_pursuit::OvertakeOverrideContractKind::SPEED_ONLY_V2);
  ASSERT_TRUE(contract->lateral_offsets.empty());
  ASSERT_EQ(contract->speed_caps.size(), 1U);
  const double target_speed_mps = simple_pure_pursuit::applyOvertakeSpeedCap(
      4.0, std::optional<double>{contract->speed_caps.front()});
  const double acceleration_mps2 =
      simple_pure_pursuit::proportionalLongitudinalAcceleration(
          target_speed_mps, 4.0, 1.0);

  EXPECT_NEAR(target_speed_mps, 0.5, 1.0e-9);
  EXPECT_LT(acceleration_mps2, 0.0);
}

TEST(OvertakeOverrideContract,
     SpeedOnlyV2MalformedPayloadRetainsCapWithoutLateralTrajectory) {
  simple_pure_pursuit::OvertakeSpeedOnlyFailClosedLatch latch;
  const auto valid_v2 = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 11.0F, 0.0F, 2.0F, 42.0F, 0.5F});
  ASSERT_TRUE(valid_v2.has_value());
  latch.observeValid(valid_v2.value());

  const auto malformed = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 11.0F, 0.0F, 2.0F, 42.0F, 0.0F});
  EXPECT_FALSE(malformed.has_value());
  const auto &retained = latch.retained();
  ASSERT_TRUE(retained.has_value());
  EXPECT_TRUE(retained->lateral_offsets.empty());
  ASSERT_EQ(retained->speed_caps.size(), 1U);
  EXPECT_NEAR(retained->speed_caps.front(), 0.5, 1.0e-9);
  EXPECT_NEAR(simple_pure_pursuit::applyOvertakeSpeedCap(
                  4.0, std::optional<double>{retained->speed_caps.front()}),
              0.5, 1.0e-9);
}

TEST(OvertakeOverrideContract,
     SpeedOnlyV2TimeoutRetainsCapWithoutLateralTrajectory) {
  simple_pure_pursuit::OvertakeSpeedOnlyFailClosedLatch latch;
  const auto valid_v2 = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 11.0F, 0.0F, 2.0F, 43.0F, 0.4F});
  ASSERT_TRUE(valid_v2.has_value());
  latch.observeValid(valid_v2.value());

  // timeout中に新しいpayloadが無くても、v2だけは縦capを保持する。
  const auto &retained = latch.retained();
  ASSERT_TRUE(retained.has_value());
  EXPECT_TRUE(retained->lateral_offsets.empty());
  ASSERT_EQ(retained->speed_caps.size(), 1U);
  EXPECT_NEAR(retained->speed_caps.front(), static_cast<double>(0.4F), 1.0e-9);
  EXPECT_NEAR(simple_pure_pursuit::applyOvertakeSpeedCap(
                  4.0, std::optional<double>{retained->speed_caps.front()}),
              static_cast<double>(0.4F), 1.0e-9);
}

TEST(OvertakeOverrideContract, V1AndExplicitInactiveClearSpeedOnlyLatch) {
  simple_pure_pursuit::OvertakeSpeedOnlyFailClosedLatch latch;
  const auto valid_v2 = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 11.0F, 0.0F, 2.0F, 42.0F, 0.5F});
  ASSERT_TRUE(valid_v2.has_value());
  latch.observeValid(valid_v2.value());

  const auto v1 = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 7.0F, 1.0F, 0.3F, 2.0F, 1.0F, 43.0F});
  ASSERT_TRUE(v1.has_value());
  latch.observeValid(v1.value());
  EXPECT_FALSE(latch.retained().has_value());

  latch.observeValid(valid_v2.value());
  const auto inactive = simple_pure_pursuit::parseOvertakeOverrideContract(
      {1.0F, 0.0F, 0.0F, 1.0F, 44.0F});
  ASSERT_TRUE(inactive.has_value());
  latch.observeValid(inactive.value());
  EXPECT_FALSE(latch.retained().has_value());
}

TEST(Lookahead, SpeedBasedDistanceMatchesExistingFormula) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;

  EXPECT_NEAR(
      simple_pure_pursuit::speedBasedLookaheadDistance(9.5, 0.0, params), 8.25,
      1.0e-9);
}

TEST(Lookahead, CurrentSpeedPreventsSuddenLookaheadShrink) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;

  EXPECT_NEAR(
      simple_pure_pursuit::speedBasedLookaheadDistance(2.5, 9.5, params), 8.25,
      1.0e-9);
}

TEST(Lookahead, CurvatureDisabledKeepsSpeedBasedDistance) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;
  params.curvature_adaptive_enabled = false;
  params.curvature_min_distance = 3.5;
  params.curvature_sensitivity = 8.0;

  EXPECT_NEAR(
      simple_pure_pursuit::adaptiveLookaheadDistance(9.5, 0.0, 0.20, params),
      8.25, 1.0e-9);
}

TEST(Lookahead, CurvatureShortensDistanceButRespectsMinimum) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;
  params.curvature_adaptive_enabled = true;
  params.curvature_min_distance = 3.5;
  params.curvature_sensitivity = 8.0;

  const double moderate_curve =
      simple_pure_pursuit::adaptiveLookaheadDistance(9.5, 0.0, 0.10, params);
  EXPECT_LT(moderate_curve, 8.25);
  EXPECT_GT(moderate_curve, 3.5);

  EXPECT_NEAR(
      simple_pure_pursuit::adaptiveLookaheadDistance(9.5, 0.0, 1.00, params),
      3.5, 1.0e-9);
}

TEST(Lookahead, ZeroSpeedStillUsesFiniteMinimumDistance) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;
  params.curvature_adaptive_enabled = true;
  params.curvature_min_distance = 2.0;
  params.curvature_sensitivity = 8.0;

  EXPECT_NEAR(
      simple_pure_pursuit::adaptiveLookaheadDistance(0.0, 0.0, 0.10, params),
      3.5, 1.0e-9);
}

TEST(Lookahead, SmoothingBlendsPreviousAndDesiredDistance) {
  EXPECT_NEAR(
      simple_pure_pursuit::smoothLookaheadDistance(4.0, 8.0, true, 0.25), 7.0,
      1.0e-9);
  EXPECT_NEAR(
      simple_pure_pursuit::smoothLookaheadDistance(4.0, 8.0, false, 0.25), 4.0,
      1.0e-9);
}

TEST(Lookahead, ForwardTrajectoryIndexSkipsCurrentAnchor) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 0.0));
  trajectory.points.push_back(makePointWithSpeed(0.1, 0.0, 0.0, 9.5));
  trajectory.points.push_back(makePointWithSpeed(0.6, 0.0, 0.0, 9.5));

  const auto idx =
      simple_pure_pursuit::selectForwardTrajectoryIndex(trajectory, 0, 0.25);

  EXPECT_EQ(idx, 2U);
  EXPECT_NEAR(trajectory.points.at(idx).longitudinal_velocity_mps, 9.5, 1.0e-9);
}

TEST(Lookahead, ForwardTrajectoryIndexFallsBackToNearestAtEnd) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 8.0));

  const auto idx =
      simple_pure_pursuit::selectForwardTrajectoryIndex(trajectory, 0, 0.25);

  EXPECT_EQ(idx, 0U);
}

TEST(Lookahead, MpcHorizonVelocityCapSkipsZeroIndexAnchor) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 0.0));
  trajectory.points.push_back(makePointWithSpeed(0.1, 0.0, 0.0, 9.5));
  trajectory.points.push_back(makePointWithSpeed(0.6, 0.0, 0.0, 9.5));

  const auto idx = simple_pure_pursuit::selectMpcHorizonVelocityCapIndex(
      trajectory, 0, 0.25, true);

  EXPECT_EQ(idx, 2U);
  EXPECT_NEAR(trajectory.points.at(idx).longitudinal_velocity_mps, 9.5, 1.0e-9);
}

TEST(Lookahead, MpcHorizonVelocityCapKeepsForwardNearestIndex) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 9.5));
  trajectory.points.push_back(makePointWithSpeed(0.6, 0.0, 0.0, 4.0));
  trajectory.points.push_back(makePointWithSpeed(1.2, 0.0, 0.0, 9.5));

  const auto idx = simple_pure_pursuit::selectMpcHorizonVelocityCapIndex(
      trajectory, 1, 0.25, true);

  EXPECT_EQ(idx, 1U);
  EXPECT_NEAR(trajectory.points.at(idx).longitudinal_velocity_mps, 4.0, 1.0e-9);
}

TEST(Lookahead, MpcHorizonVelocityCapKeepsSolverZeroIndex) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 2.0));
  trajectory.points.push_back(makePointWithSpeed(0.6, 0.0, 0.0, 9.5));

  const auto idx = simple_pure_pursuit::selectMpcHorizonVelocityCapIndex(
      trajectory, 0, 0.25, false);

  EXPECT_EQ(idx, 0U);
  EXPECT_NEAR(trajectory.points.at(idx).longitudinal_velocity_mps, 2.0, 1.0e-9);
}

TEST(LongitudinalOverride, ImmediateSpeedCapRequestsBrakingInSameCycle) {
  constexpr double current_speed_mps = 4.0;
  constexpr double planner_speed_cap_mps = 0.2;
  constexpr double speed_proportional_gain = 1.0;

  const double target_speed_mps = simple_pure_pursuit::applyOvertakeSpeedCap(
      current_speed_mps, std::optional<double>{planner_speed_cap_mps});
  const double acceleration_mps2 =
      simple_pure_pursuit::proportionalLongitudinalAcceleration(
          target_speed_mps, current_speed_mps, speed_proportional_gain);

  EXPECT_LT(target_speed_mps, current_speed_mps);
  EXPECT_NEAR(target_speed_mps, planner_speed_cap_mps, 1.0e-9);
  EXPECT_LT(acceleration_mps2, 0.0);
}

TEST(HorizonContract, UnauthorizedAbortRecoverySolverHorizonIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 42U, true, std::optional<double>{10.0}, 10.1, 0.5, true,
      "solver_prediction", 7, 42U);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "mpc_horizon_not_authorized");
}

TEST(HorizonContract, MandatoryAbortRecoverySolverHorizonIsUsable) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 42U, true, std::optional<double>{10.0}, 10.1, 0.5, true,
      "solver_prediction", 7, 42U, true, true, true, true);

  EXPECT_TRUE(result.usable);
  EXPECT_EQ(result.reason, "fresh");
}

TEST(HorizonContract, AuthorizedButNonMandatoryAbortRecoveryIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 42U, true, std::optional<double>{10.0}, 10.1, 0.5, true,
      "solver_prediction", 7, 42U, true, true, false, false);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "mpc_horizon_abort_not_mandatory");
}

TEST(HorizonContract, MismatchedGenerationFallsBackFromMpcHorizon) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 42U, true, std::optional<double>{10.0}, 10.1, 0.5, true,
      "solver_prediction", 7, 41U, true, true, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "mpc_horizon_contract_generation_mismatch");
}

TEST(HorizonContract, MissingMetadataFallsBackFromMpcHorizon) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 42U, false, std::nullopt, 10.1, 0.5, false, "unknown", 0,
      0U);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "mpc_horizon_contract_missing");
}

TEST(HorizonContract, InactiveOverrideRejectsNonzeroSolverContract) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, false, 0, 0U, true, std::optional<double>{10.0}, 10.1, 0.5, true,
      "solver_prediction", 7, 42U);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "mpc_horizon_contract_inactive_nonzero");
}

TEST(HorizonContract, StampMismatchFallsBackFromMpcHorizon) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 42U, true, std::optional<double>{10.0}, 10.1, 0.5, false,
      "solver_prediction", 7, 42U, true, true, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "mpc_horizon_contract_stamp_mismatch");
}

TEST(HorizonContract, SourceModeAndStalenessAreRejected) {
  const auto source = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 42U, true, std::optional<double>{10.0}, 10.1, 0.5, true,
      "neutral_reference", 7, 42U, true, true, true, true);
  const auto mode = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 42U, true, std::optional<double>{10.0}, 10.1, 0.5, true,
      "solver_prediction", 6, 42U, true, true, true, true);
  const auto stale = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 42U, true, std::optional<double>{9.0}, 10.1, 0.5, true,
      "solver_prediction", 7, 42U, true, true, true, true);

  EXPECT_EQ(source.reason, "mpc_horizon_contract_source");
  EXPECT_EQ(mode.reason, "mpc_horizon_contract_mode_mismatch");
  EXPECT_EQ(stale.reason, "mpc_horizon_contract_stale");
}

TEST(HorizonContract, LegacyGenerationAndDisabledStrictGateBehaveSafely) {
  const auto legacy = simple_pure_pursuit::evaluateMpcHorizonContract(
      true, true, 7, 0U, true, std::optional<double>{10.0}, 10.1, 0.5, true,
      "solver_prediction", 7, 0U, true, true, true, true);
  const auto disabled = simple_pure_pursuit::evaluateMpcHorizonContract(
      false, true, 7, 42U, false, std::nullopt, 10.1, 0.5, false, "unknown", 0,
      0U);

  EXPECT_FALSE(legacy.usable);
  EXPECT_EQ(legacy.reason, "mpc_horizon_contract_generation_mismatch");
  EXPECT_TRUE(disabled.usable);
  EXPECT_EQ(disabled.reason, "fresh");
}

TEST(Lookahead, StraightTrajectoryCurvatureIsZero) {
  Trajectory trajectory;
  for (int i = 0; i < 6; ++i) {
    trajectory.points.push_back(makePoint(static_cast<double>(i), 0.0, 0.0));
  }

  EXPECT_NEAR(
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 5, 0.5),
      0.0, 1.0e-9);
}

TEST(Lookahead, CurvatureReadIndexReportsTheLastActuallyReadPoint) {
  Trajectory trajectory;
  for (int index = 0; index < 6; ++index) {
    trajectory.points.push_back(
        makePoint(static_cast<double>(index), 0.0, 0.0));
  }

  std::size_t unsigned_last_read = 0U;
  std::size_t signed_last_read = 0U;
  (void)simple_pure_pursuit::estimateTrajectoryCurvature(
      trajectory, 0U, 2.0, 0.5, &unsigned_last_read);
  (void)simple_pure_pursuit::estimateSignedTrajectoryCurvature(
      trajectory, 0U, 2.0, 0.5, &signed_last_read);

  EXPECT_EQ(unsigned_last_read, 3U);
  EXPECT_EQ(signed_last_read, 3U);
}

TEST(Lookahead, CurvedTrajectoryCurvatureIsPositive) {
  constexpr double radius_m = 10.0;
  constexpr double step_rad = 0.10;

  Trajectory trajectory;
  for (int i = 0; i < 8; ++i) {
    const double theta = step_rad * static_cast<double>(i);
    trajectory.points.push_back(makePoint(
        radius_m * std::sin(theta), radius_m * (1.0 - std::cos(theta)), theta));
  }

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 6, 0.5);

  EXPECT_GT(curvature, 0.05);
  EXPECT_LT(curvature, 0.15);
}

TEST(Lookahead, CurvatureUsesGeometryEvenWhenYawIsStraight) {
  constexpr double radius_m = 10.0;
  constexpr double step_rad = 0.10;

  Trajectory trajectory;
  for (int i = 0; i < 8; ++i) {
    const double theta = step_rad * static_cast<double>(i);
    trajectory.points.push_back(makePoint(
        radius_m * std::sin(theta), radius_m * (1.0 - std::cos(theta)), 0.0));
  }

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 6.0, 0.5);

  EXPECT_GT(curvature, 0.05);
  EXPECT_LT(curvature, 0.15);
}

TEST(Lookahead, CurvatureUsesHeadingChangeEvenWhenGeometryIsStraight) {
  Trajectory trajectory;
  for (int i = 0; i < 8; ++i) {
    const double x = static_cast<double>(i);
    const double yaw = 0.10 * static_cast<double>(i);
    trajectory.points.push_back(makePoint(x, 0.0, yaw));
  }

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 6.0, 0.5);

  EXPECT_GT(curvature, 0.09);
  EXPECT_LT(curvature, 0.11);
}

TEST(Lookahead, CurvatureDoesNotRecognizeCornerOutsideWindow) {
  const auto trajectory = makeStraightThenArcTrajectory();

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 4.0, 0.5);

  EXPECT_NEAR(curvature, 0.0, 1.0e-9);
}

TEST(Lookahead, CurvatureRecognizesCornerAheadWithinWindow) {
  const auto trajectory = makeStraightThenArcTrajectory();

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 6.0, 0.5);

  EXPECT_GT(curvature, 0.05);
  EXPECT_LT(curvature, 0.15);
}

TEST(Lookahead, SignedCurvatureKeepsLeftRightDirection) {
  const double left_curvature =
      simple_pure_pursuit::estimateSignedTrajectoryCurvature(
          makeArcTrajectory(1.0), 0, 6.0, 0.5);
  const double right_curvature =
      simple_pure_pursuit::estimateSignedTrajectoryCurvature(
          makeArcTrajectory(-1.0), 0, 6.0, 0.5);

  EXPECT_GT(left_curvature, 0.05);
  EXPECT_LT(left_curvature, 0.15);
  EXPECT_LT(right_curvature, -0.05);
  EXPECT_GT(right_curvature, -0.15);
}

TEST(Lookahead, AdaptiveLookaheadReturnsBaseForNaNAndInfCurvature) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;
  params.curvature_adaptive_enabled = true;
  params.curvature_min_distance = 3.5;
  params.curvature_sensitivity = 8.0;

  const double base =
      simple_pure_pursuit::speedBasedLookaheadDistance(9.5, 0.0, params);
  EXPECT_NEAR(simple_pure_pursuit::adaptiveLookaheadDistance(
                  9.5, 0.0, std::numeric_limits<double>::quiet_NaN(), params),
              base, 1.0e-9);
  EXPECT_NEAR(simple_pure_pursuit::adaptiveLookaheadDistance(
                  9.5, 0.0, std::numeric_limits<double>::infinity(), params),
              base, 1.0e-9);
}

TEST(Lookahead, InvalidAndShortTrajectoryFallsBackToZeroCurvature) {
  Trajectory short_trajectory;
  short_trajectory.points.push_back(makePoint(0.0, 0.0, 0.0));
  short_trajectory.points.push_back(makePoint(1.0, 0.0, 0.0));

  EXPECT_NEAR(simple_pure_pursuit::estimateTrajectoryCurvature(short_trajectory,
                                                               0, 5, 0.5),
              0.0, 1.0e-9);

  Trajectory repeated_trajectory;
  repeated_trajectory.points.push_back(makePoint(0.0, 0.0, 0.0));
  repeated_trajectory.points.push_back(makePoint(0.0, 0.0, 0.5));
  repeated_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0));

  EXPECT_NEAR(simple_pure_pursuit::estimateTrajectoryCurvature(
                  repeated_trajectory, 0, 5, 0.5),
              0.0, 1.0e-9);
}

TEST(Safety, MissingInputsAreInvalid) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      std::nullopt, std::nullopt, 10.0, 0.2, 0.5);

  EXPECT_FALSE(result.fresh);
  EXPECT_EQ(result.reason, "missing_odom");
}

TEST(Safety, StaleOdometryIsInvalid) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      9.0, 9.9, 10.0, 0.2, 0.5);

  EXPECT_FALSE(result.fresh);
  EXPECT_EQ(result.reason, "stale_odom");
  EXPECT_NEAR(result.ages.odom_age_sec, 1.0, 1.0e-9);
}

TEST(Safety, StaleTrajectoryIsInvalid) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      9.9, 9.0, 10.0, 0.2, 0.5);

  EXPECT_FALSE(result.fresh);
  EXPECT_EQ(result.reason, "stale_trajectory");
  EXPECT_NEAR(result.ages.trajectory_age_sec, 1.0, 1.0e-9);
}

TEST(Safety, FreshInputsAreValid) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      9.9, 9.7, 10.0, 0.2, 0.5);

  EXPECT_TRUE(result.fresh);
  EXPECT_EQ(result.reason, "fresh");
  EXPECT_NEAR(result.ages.odom_age_sec, 0.1, 1.0e-9);
  EXPECT_NEAR(result.ages.trajectory_age_sec, 0.3, 1.0e-9);
}

TEST(Safety, NegativeMaxAgeDisablesThatFreshnessCheck) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      1.0, 1.0, 10.0, -1.0, -1.0);

  EXPECT_TRUE(result.fresh);
}

TEST(Safety, DisabledMpcHorizonIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      false, 9.99, 10.0, 0.15, 20, 5, 0.1, 2.0, 6.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "disabled");
}

TEST(Safety, StaleMpcHorizonIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.0, 10.0, 0.15, 20, 5, 0.1, 2.0, 6.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "stale");
}

TEST(Safety, ShortMpcHorizonIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.99, 10.0, 0.15, 2, 5, 0.1, 2.0, 6.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "short");
}

TEST(Safety, FarMpcHorizonStartIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.99, 10.0, 0.15, 20, 5, 3.0, 2.0, 6.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "start_distance");
}

TEST(Safety, ShortArcMpcHorizonIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.99, 10.0, 0.15, 20, 5, 0.1, 2.0, 1.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "short_arc");
}

TEST(Safety, FreshMpcHorizonIsUsable) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.99, 10.0, 0.15, 20, 5, 0.1, 2.0, 6.0, 2.0, true, true);

  EXPECT_TRUE(result.usable);
  EXPECT_EQ(result.reason, "fresh");
  EXPECT_NEAR(result.age_sec, 0.01, 1.0e-9);
}

TEST(DelayCompensation, ZeroDelayKeepsInputPose) {
  const simple_pure_pursuit::EgoControlState state{1.0, 2.0, 0.3, 5.0};
  const auto prediction = simple_pure_pursuit::predictDelayedPose(
      state, 0.1, 0.2, 0.0, 0.02, 0.30, 1.087);

  EXPECT_FALSE(prediction.shifted);
  EXPECT_EQ(prediction.prediction_steps, 0);
  EXPECT_NEAR(prediction.x, 1.0, 1.0e-9);
  EXPECT_NEAR(prediction.y, 2.0, 1.0e-9);
  EXPECT_NEAR(prediction.yaw, 0.3, 1.0e-9);
  EXPECT_NEAR(prediction.applied_steering_rad, 0.1, 1.0e-9);
}

TEST(DelayCompensation, StraightMotionAdvancesByDelayDistance) {
  const simple_pure_pursuit::EgoControlState state{0.0, 0.0, 0.0, 5.0};
  const auto prediction = simple_pure_pursuit::predictDelayedPose(
      state, 0.0, 0.0, 0.20, 0.02, 0.30, 1.087);

  EXPECT_TRUE(prediction.shifted);
  EXPECT_EQ(prediction.prediction_steps, 10);
  EXPECT_NEAR(prediction.x, 1.0, 1.0e-9);
  EXPECT_NEAR(prediction.y, 0.0, 1.0e-9);
  EXPECT_NEAR(prediction.yaw, 0.0, 1.0e-9);
}

TEST(DelayCompensation, SteeringLagMovesTowardPreviousCommand) {
  const double next_steer =
      simple_pure_pursuit::predictLaggedSteering(0.0, 0.4, 0.10, 0.30);

  EXPECT_GT(next_steer, 0.0);
  EXPECT_LT(next_steer, 0.4);
}

TEST(DelayCompensation, ZeroVelocityDoesNotDriftYaw) {
  const simple_pure_pursuit::EgoControlState state{0.0, 0.0, 1.0, 0.0};
  const auto prediction = simple_pure_pursuit::predictDelayedPose(
      state, 0.5, 0.5, 0.20, 0.02, 0.30, 1.087);

  EXPECT_TRUE(prediction.shifted);
  EXPECT_NEAR(prediction.x, 0.0, 1.0e-9);
  EXPECT_NEAR(prediction.y, 0.0, 1.0e-9);
  EXPECT_NEAR(prediction.yaw, 1.0, 1.0e-9);
}

TEST(Ay0BaseCapture, CopiesExactPreAdaptationSourceWithoutHashing) {
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 10;
  trajectory.points.resize(3U);
  for (std::size_t index = 0U; index < trajectory.points.size(); ++index) {
    auto &point = trajectory.points[index];
    point.pose.position.x = static_cast<double>(index);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(index + 1U);
  }
  simple_pure_pursuit::Ay0BaseCaptureInput input;
  input.trajectory = &trajectory;
  input.nearest_source_index = 1U;
  input.source_kind =
      overtake_transport_contract::c002ay0::kFixedBaseSourceReferenceTrajectory;
  input.source_generation = 7U;
  input.record_stamp = {10, 1U};
  input.lease_valid_until = {11, 0U};
  input.session_generation = 8U;
  input.session_nonce = 9U;
  input.race_arm_epoch = 10U;
  input.controller_instance_id = 11U;
  input.controller_sequence = 12U;
  input.base_lease_id = 13U;
  input.controller_implementation_sha256[0] = 1U;
  input.controller_config_sha256[0] = 2U;

  overtake_transport_contract::c002ay0::FixedBaseRecord record;
  EXPECT_EQ(simple_pure_pursuit::buildAy0FixedBaseRecord(input, record),
            simple_pure_pursuit::Ay0BaseCaptureResult::kBuilt);
  EXPECT_EQ(record.point_count, 3U);
  EXPECT_EQ(record.base_original_point_count, 3U);
  EXPECT_EQ(record.nearest_source_index, 1U);
  EXPECT_DOUBLE_EQ(record.points[2].position_x_m, 2.0);
  EXPECT_FLOAT_EQ(record.points[2].longitudinal_velocity_mps, 3.0F);
}

TEST(Ay0BaseCapture, RejectsOversizeWithoutTruncating) {
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points.resize(
      overtake_transport_contract::c002ay0::kMaxCartesianPoints + 1U);
  simple_pure_pursuit::Ay0BaseCaptureInput input;
  input.trajectory = &trajectory;
  overtake_transport_contract::c002ay0::FixedBaseRecord record;

  EXPECT_EQ(simple_pure_pursuit::buildAy0FixedBaseRecord(input, record),
            simple_pure_pursuit::Ay0BaseCaptureResult::kPointLimit);
  EXPECT_EQ(record.point_count, 0U);
}

TEST(Ay0BaseCapture, CanonicallyCapturesBoundedWindowWithAbsoluteIndices) {
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 10;
  trajectory.points.resize(350U);
  for (std::size_t index = 0U; index < trajectory.points.size(); ++index) {
    auto &point = trajectory.points[index];
    point.time_from_start.nanosec =
        static_cast<std::uint32_t>((index + 1U) * 1000000U);
    point.pose.position.x = 0.1 * static_cast<double>(index);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 1.0F;
  }
  simple_pure_pursuit::Ay0BaseCaptureInput input;
  input.trajectory = &trajectory;
  input.nearest_source_index = 175U;
  input.source_kind =
      overtake_transport_contract::c002ay0::kFixedBaseSourceReferenceTrajectory;
  input.source_generation = 7U;
  input.record_stamp = {10, 1U};
  input.lease_valid_until = {11, 0U};
  input.race_arm_epoch = 10U;
  input.controller_instance_id = 11U;
  input.controller_sequence = 12U;
  input.base_lease_id = 13U;
  input.controller_implementation_sha256[0] = 1U;
  input.controller_config_sha256[0] = 2U;

  multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot snapshot;
  ASSERT_EQ(simple_pure_pursuit::buildAy0BaseSnapshot(input, snapshot),
            simple_pure_pursuit::Ay0BaseCaptureResult::kBuilt);
  EXPECT_EQ(snapshot.canonical_algorithm_version, 2U);
  EXPECT_EQ(snapshot.base_original_point_count, 350U);
  EXPECT_EQ(snapshot.first_source_index, 94U);
  EXPECT_EQ(snapshot.last_source_index, 349U);
  EXPECT_EQ(snapshot.nearest_source_index, 175U);
  ASSERT_EQ(snapshot.base_points.size(), 256U);
  EXPECT_DOUBLE_EQ(snapshot.base_points.front().position_x_m, 9.4);
  EXPECT_DOUBLE_EQ(snapshot.base_points.back().position_x_m, 34.9);
  const auto nearest_local_index =
      snapshot.nearest_source_index - snapshot.first_source_index;
  ASSERT_LT(nearest_local_index, snapshot.base_points.size());
  EXPECT_DOUBLE_EQ(snapshot.base_points[nearest_local_index].position_x_m,
                   trajectory.points[input.nearest_source_index]
                       .pose.position.x);
  EXPECT_EQ(overtake_transport_contract::c002ay0::validateBaseSnapshotV1(
                snapshot),
            overtake_transport_contract::c002ay0::ValidationError::NONE);

  auto changed_mapping = snapshot;
  ++changed_mapping.first_source_index;
  EXPECT_EQ(overtake_transport_contract::c002ay0::validateBaseSnapshotV1(
                changed_mapping),
            overtake_transport_contract::c002ay0::ValidationError::SOURCE_WINDOW_INVALID);

  auto forged_source = snapshot;
  ++forged_source.base_source_sha256[0];
  const auto forged_snapshot =
      overtake_transport_contract::c002ay0::canonicalizeBaseSnapshotV1(
          forged_source);
  EXPECT_EQ(forged_snapshot.error,
            overtake_transport_contract::c002ay0::ValidationError::DIGEST_MISMATCH);

  auto wrong_algorithm = snapshot;
  wrong_algorithm.canonical_algorithm_version = 1U;
  EXPECT_EQ(overtake_transport_contract::c002ay0::validateBaseSnapshotV1(
                wrong_algorithm),
            overtake_transport_contract::c002ay0::ValidationError::SOURCE_WINDOW_INVALID);
}

TEST(Ay0BaseCapture, KeepsCanonicalWindowPolicyAtLongTrajectoryBoundaries) {
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 10;
  trajectory.points.resize(350U);
  for (std::size_t index = 0U; index < trajectory.points.size(); ++index) {
    auto &point = trajectory.points[index];
    point.time_from_start.nanosec =
        static_cast<std::uint32_t>((index + 1U) * 1000000U);
    point.pose.position.x = 0.1 * static_cast<double>(index);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 1.0F;
  }

  for (const auto nearest : {0U, 175U, 349U}) {
    simple_pure_pursuit::Ay0BaseCaptureInput input;
    input.trajectory = &trajectory;
    input.nearest_source_index = nearest;
    input.source_kind = overtake_transport_contract::c002ay0::
        kFixedBaseSourceReferenceTrajectory;
    input.source_generation = 7U;
    input.record_stamp = {10, 1U};
    input.lease_valid_until = {11, 0U};
    input.race_arm_epoch = 10U;
    input.controller_instance_id = 11U;
    input.controller_sequence = 12U;
    input.base_lease_id = 13U;
    input.controller_implementation_sha256[0] = 1U;
    input.controller_config_sha256[0] = 2U;

    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot snapshot;
    ASSERT_EQ(simple_pure_pursuit::buildAy0BaseSnapshot(input, snapshot),
              simple_pure_pursuit::Ay0BaseCaptureResult::kBuilt);
    const auto expected_first = std::min(nearest, 350U - 256U);
    EXPECT_EQ(snapshot.first_source_index, expected_first);
    EXPECT_EQ(snapshot.last_source_index, expected_first + 255U);
    const auto nearest_local = nearest - expected_first;
    ASSERT_LT(nearest_local, snapshot.base_points.size());
    EXPECT_DOUBLE_EQ(snapshot.base_points[nearest_local].position_x_m,
                     trajectory.points[nearest].pose.position.x);
    EXPECT_EQ(overtake_transport_contract::c002ay0::validateBaseSnapshotV1(
                  snapshot),
              overtake_transport_contract::c002ay0::ValidationError::NONE);
  }
}
