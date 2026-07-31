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
#include <overtake_planner/cartesian_trackability_evaluator.hpp>
#include <overtake_planner/pp_core_exact_snapshot.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>

namespace simple_pure_pursuit {

struct ProductionPurePursuitGolden {
  std::size_t nearest_source_index{0U};
  std::size_t selected_lookahead_source_index{0U};
  bool lookahead_endpoint_fallback{false};
  double lookahead_distance_m{0.0};
  double geometric_steering_rad{0.0};
  double curvature_feedforward_rad{0.0};
  double raw_steering_rad{0.0};
  double requested_steering_rad{0.0};
  double bounded_steering_rad{0.0};
  double limiter_reference_rad{0.0};
  double requested_rate_radps{0.0};
  double bounded_rate_radps{0.0};
  bool angle_limited{false};
  bool rate_limited{false};
};

class PurePursuitExactGoldenAccess {
public:
  static ProductionPurePursuitGolden
  compute(SimplePurePursuit &node, const Trajectory &trajectory,
          std::size_t nearest_source_index, double control_x_m,
          double control_y_m, double control_yaw_rad, double speed_mps,
          double steering_reference_rad) {
    SimplePurePursuit::ControlTrajectoryContext context;
    context.valid = true;
    context.base_trajectory = &trajectory;
    context.trajectory = &trajectory;
    context.base_nearest_index = nearest_source_index;
    context.nearest_index = nearest_source_index;
    SimplePurePursuit::ControlPosePrediction pose;
    pose.position.x = control_x_m;
    pose.position.y = control_y_m;
    pose.yaw = control_yaw_rad;
    pose.velocity_mps = speed_mps;
    pose.current_steering_rad = steering_reference_rad;
    SimplePurePursuit::LongitudinalCommand longitudinal;
    longitudinal.target_speed_mps = speed_mps;
    longitudinal.current_speed_mps = speed_mps;
    const auto lateral =
        node.computeLateralCommand(context, pose, longitudinal);
    return {
        nearest_source_index,
        lateral.lookahead_selected_trajectory_index,
        lateral.lookahead_endpoint_fallback,
        lateral.lookahead_distance_m,
        lateral.pure_pursuit_steering_tire_angle_rad,
        lateral.curvature_feedforward_steering_rad,
        lateral.raw_steering_tire_angle_rad,
        lateral.requested_output_steering_tire_angle_rad,
        lateral.steering_tire_angle_rad,
        lateral.limiter_reference_steering_rad,
        lateral.requested_steering_tire_rotation_rate_radps,
        lateral.steering_tire_rotation_rate_radps,
        lateral.steering_angle_limited,
        lateral.steering_rate_limited,
    };
  }
};

} // namespace simple_pure_pursuit

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

simple_pure_pursuit::PurePursuitExactSnapshotInput makeExactExecutionSnapshot(
    const multi_purpose_mpc_ros_msgs::msg::ControllerCommandEnvelope &command,
    const simple_pure_pursuit::ControllerExecutionWitnessInput &input,
    std::size_t nearest_index, std::size_t lookahead_index) {
  (void)nearest_index;
  simple_pure_pursuit::PurePursuitExactSnapshotInput exact;
  exact.enabled = true;
  exact.base_source_binding_valid = true;
  exact.base_source_key.canonical_algorithm_version =
      multi_purpose_mpc_ros_msgs::msg::FreeRunSourceKey::CANONICAL_ALGORITHM_V1;
  exact.base_source_key.baseline_instance_id = 88U;
  exact.base_source_key.controller_instance_id = command.producer_instance_id;
  exact.base_source_key.source_generation = 4U;
  exact.base_source_key.source_stamp = input.reference_stamp;
  exact.base_source_key.original_point_count =
      static_cast<std::uint32_t>(input.base_trajectory.points.size());
  exact.base_source_key.baseline_reference_sha256.fill(0x11U);
  exact.base_source_key.controller_implementation_sha256.fill(0x22U);
  exact.base_source_key.controller_config_sha256.fill(0x33U);
  exact.control_pose_stamp = command.header.stamp;
  exact.diagnostic_lease_duration_sec = 0.05;
  exact.selected_lookahead_trajectory_index = lookahead_index;
  exact.control_speed_mps = 1.0;
  exact.resolved_lookahead_distance_m = 2.0;
  exact.geometric_steering_tire_angle_rad = 0.08;
  exact.curvature_feedforward_steering_rad = 0.01;
  exact.raw_steering_tire_angle_rad = 0.09;
  exact.requested_output_steering_tire_angle_rad = 0.1;
  exact.bounded_steering_tire_angle_rad = 0.1;
  exact.requested_steering_tire_rotation_rate_radps = 0.2;
  exact.bounded_steering_tire_rotation_rate_radps = 0.2;
  exact.limiter_reference_valid = true;
  exact.limiter_reference_steering_rad = 0.098;
  exact.command_dt_sec = 0.01;
  exact.wheelbase_m = 1.087;
  exact.steering_output_gain = 1.0;
  exact.hard_steering_angle_limit_rad = 0.64;
  exact.hard_steering_rate_limit_radps = 128.0;
  exact.required_horizon_end_trajectory_index = lookahead_index;
  return exact;
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

TEST(SteeringCommandLimiter, AppliesAngleThenRateThenAngleClamp) {
  const auto result =
      simple_pure_pursuit::boundSteeringCommand(0.60, 0.10, 0.01, 0.366, 8.0);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.angle_limited);
  EXPECT_TRUE(result.rate_limited);
  EXPECT_NEAR(result.bounded_angle_rad, 0.18, 1.0e-12);
  EXPECT_NEAR(result.bounded_rate_radps, 8.0, 1.0e-12);
  EXPECT_GT(result.requested_rate_radps, result.bounded_rate_radps);
}

TEST(SteeringCommandLimiter, PreservesCommandAlreadyInsideBothLimits) {
  const auto result =
      simple_pure_pursuit::boundSteeringCommand(0.15, 0.10, 0.01, 0.366, 8.0);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.angle_limited);
  EXPECT_FALSE(result.rate_limited);
  EXPECT_NEAR(result.bounded_angle_rad, 0.15, 1.0e-12);
  EXPECT_NEAR(result.bounded_rate_radps, 5.0, 1.0e-12);
}

TEST(SteeringCommandLimiter, AllowsFullVehicleRangeInOneNominalTick) {
  const auto result =
      simple_pure_pursuit::boundSteeringCommand(0.64, -0.64, 0.01, 0.64, 128.0);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.angle_limited);
  EXPECT_FALSE(result.rate_limited);
  EXPECT_NEAR(result.bounded_angle_rad, 0.64, 1.0e-12);
  EXPECT_NEAR(result.bounded_rate_radps, 128.0, 1.0e-12);
}

TEST(SteeringCommandLimiter, RejectsInvalidTimingOrLimits) {
  EXPECT_FALSE(
      simple_pure_pursuit::boundSteeringCommand(0.1, 0.0, 0.0, 0.366, 8.0)
          .valid);
  EXPECT_FALSE(
      simple_pure_pursuit::boundSteeringCommand(
          std::numeric_limits<double>::quiet_NaN(), 0.0, 0.01, 0.366, 8.0)
          .valid);
  EXPECT_FALSE(
      simple_pure_pursuit::boundSteeringCommand(0.1, 0.0, 0.01, 0.0, 8.0)
          .valid);
}

TEST(PurePursuitExactGolden,
     ProductionControllerMatchesIndependentCartesianEvaluator) {
  unsetenv("CYCLONEDDS_URI");
  int argc = 0;
  char **argv = nullptr;
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("aw2_shadow_transport_enabled", false),
      rclcpp::Parameter("c002ay0_shadow_capture_enabled", false),
      rclcpp::Parameter("free_run_live_exact_ack_enabled", false),
      rclcpp::Parameter("pp_core_exact_snapshot_enabled", false),
      rclcpp::Parameter("use_overtake_reference_override", false),
      rclcpp::Parameter("curvature_adaptive_lookahead_enabled", false),
      rclcpp::Parameter("lookahead_gain", 0.0),
      rclcpp::Parameter("lookahead_min_distance", 0.4),
      rclcpp::Parameter("horizon_curvature_feedforward_gain", 0.0),
      rclcpp::Parameter("steering_tire_angle_gain", 1.0),
      rclcpp::Parameter("wheel_base", 1.087),
      rclcpp::Parameter("free_run_live_exact_hard_steering_limit_rad", 0.64),
      rclcpp::Parameter("free_run_live_exact_hard_steering_rate_limit_radps",
                        128.0),
      rclcpp::Parameter("steering_command_nominal_dt_sec", 0.01),
  });
  auto node = std::make_shared<simple_pure_pursuit::SimplePurePursuit>(options);

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 12;
  overtake_planner::CandidateTrajectory candidate;
  for (std::size_t index = 0U; index < 130U; ++index) {
    const double x_m = static_cast<double>(index) * 0.1;
    trajectory.points.push_back(makePointWithSpeed(x_m, 0.0, 0.0, 1.0));
    candidate.x.push_back(x_m);
    candidate.y.push_back(0.0);
    candidate.yaw.push_back(0.0);
    candidate.t.push_back(x_m);
    candidate.longitudinal_offsets_m.push_back(x_m);
    candidate.predicted_speed_mps.push_back(1.0);
    candidate.v_ref.push_back(1.0);
  }
  constexpr std::size_t nearest_source_index = 120U;
  const auto production =
      simple_pure_pursuit::PurePursuitExactGoldenAccess::compute(
          *node, trajectory, nearest_source_index, 12.5, 0.0, 0.0, 1.0, 0.0);

  overtake_planner::FrenetFrame frame;
  frame.setReference({
      {0.0, 0.0, 0.0, 0.0, 0.0, 5.0},
      {100.0, 100.0, 0.0, 0.0, 0.0, 5.0},
      {200.0, 100.0, 100.0, 1.5707963267948966, 0.0, 5.0},
  });
  overtake_planner::CartesianTrackabilityInput input;
  input.ego.valid = true;
  input.ego.x = 12.5;
  input.ego.y = 0.0;
  input.ego.yaw = 0.0;
  input.ego.v = 1.0;
  input.active_lookahead_valid = true;
  input.active_lookahead_distance_m = production.lookahead_distance_m;
  input.nearest_source_index_valid = true;
  input.nearest_source_index = production.nearest_source_index;
  input.curvature_feedforward_valid = true;
  input.curvature_feedforward_steering_rad =
      production.curvature_feedforward_rad;
  input.steering_reference_valid = true;
  input.steering_reference_angle_rad = production.limiter_reference_rad;
  input.steering_command_dt_sec = 0.01;
  overtake_planner::CartesianTrackabilityConfig config;
  config.wheelbase_m = 1.087;
  config.steering_gain = 1.0;
  config.max_steering_angle_rad = 0.64;
  config.max_steering_rate_radps = 128.0;
  config.max_yaw_tangent_error_rad = 0.1;
  config.pure_pursuit_required_arc_m = production.lookahead_distance_m;
  config.target_d_m = 0.0;
  config.target_d_deadline_arc_m = 1.0;
  const auto evaluated =
      overtake_planner::CartesianTrackabilityEvaluator(frame).evaluate(
          candidate, input, config);

  ASSERT_TRUE(evaluated.valid) << evaluated.reason;
  EXPECT_EQ(evaluated.pure_pursuit_target_source_index,
            production.selected_lookahead_source_index);
  EXPECT_NEAR(evaluated.pure_pursuit_lookahead_distance_m,
              production.lookahead_distance_m, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_geometric_steering_angle_rad,
              production.geometric_steering_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.curvature_feedforward_steering_angle_rad,
              production.curvature_feedforward_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_raw_steering_angle_rad,
              production.raw_steering_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_requested_steering_angle_rad,
              production.requested_steering_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_bounded_steering_angle_rad,
              production.bounded_steering_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_requested_steering_rate_radps,
              production.requested_rate_radps, 1.0e-12);
  EXPECT_EQ(production.angle_limited,
            std::abs(evaluated.pure_pursuit_requested_steering_angle_rad) >
                config.max_steering_angle_rad);
  EXPECT_EQ(production.rate_limited,
            std::abs(evaluated.pure_pursuit_requested_steering_rate_radps) >
                config.max_steering_rate_radps);
  node.reset();
  rclcpp::shutdown();
}

struct CurvedPurePursuitGoldenParam {
  double hard_rate_limit_radps;
  bool expect_rate_limited;
};

class CurvedPurePursuitExactGolden
    : public ::testing::TestWithParam<CurvedPurePursuitGoldenParam> {};

TEST_P(CurvedPurePursuitExactGolden,
       NonzeroFeedforwardReferenceAndLimiterMatchIndependentEvaluator) {
  unsetenv("CYCLONEDDS_URI");
  int argc = 0;
  char **argv = nullptr;
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  const auto parameter = GetParam();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("aw2_shadow_transport_enabled", false),
      rclcpp::Parameter("c002ay0_shadow_capture_enabled", false),
      rclcpp::Parameter("free_run_live_exact_ack_enabled", false),
      rclcpp::Parameter("pp_core_exact_snapshot_enabled", false),
      rclcpp::Parameter("use_overtake_reference_override", false),
      rclcpp::Parameter("curvature_adaptive_lookahead_enabled", false),
      rclcpp::Parameter("curvature_lookahead_min_arc_length", 0.1),
      rclcpp::Parameter("lookahead_gain", 0.0),
      rclcpp::Parameter("lookahead_min_distance", 0.6),
      rclcpp::Parameter("horizon_curvature_feedforward_gain", 0.5),
      rclcpp::Parameter("horizon_curvature_feedforward_max_rad", 0.2),
      rclcpp::Parameter("steering_tire_angle_gain", 1.0),
      rclcpp::Parameter("wheel_base", 1.087),
      rclcpp::Parameter("free_run_live_exact_hard_steering_limit_rad", 0.64),
      rclcpp::Parameter("free_run_live_exact_hard_steering_rate_limit_radps",
                        parameter.hard_rate_limit_radps),
      rclcpp::Parameter("steering_command_nominal_dt_sec", 0.01),
  });
  auto node = std::make_shared<simple_pure_pursuit::SimplePurePursuit>(options);

  constexpr double radius_m = 10.0;
  constexpr double theta_step_rad = 0.01;
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 12;
  overtake_planner::CandidateTrajectory candidate;
  std::vector<overtake_planner::ReferencePoint> reference;
  for (std::size_t index = 0U; index < 160U; ++index) {
    const double theta_rad = theta_step_rad * static_cast<double>(index);
    const double arc_m = radius_m * theta_rad;
    const double x_m = radius_m * std::sin(theta_rad);
    const double y_m = radius_m * (1.0 - std::cos(theta_rad));
    trajectory.points.push_back(makePointWithSpeed(x_m, y_m, theta_rad, 1.0));
    candidate.x.push_back(x_m);
    candidate.y.push_back(y_m);
    candidate.yaw.push_back(theta_rad);
    candidate.t.push_back(arc_m);
    candidate.longitudinal_offsets_m.push_back(arc_m);
    candidate.predicted_speed_mps.push_back(1.0);
    candidate.v_ref.push_back(1.0);
    reference.push_back({arc_m, x_m, y_m, theta_rad, 0.0, 5.0});
  }
  constexpr std::size_t nearest_source_index = 20U;
  constexpr double steering_reference_rad = 0.20;
  const auto &control_point =
      trajectory.points[nearest_source_index].pose.position;
  const double control_yaw_rad =
      theta_step_rad * static_cast<double>(nearest_source_index);
  const auto production =
      simple_pure_pursuit::PurePursuitExactGoldenAccess::compute(
          *node, trajectory, nearest_source_index, control_point.x,
          control_point.y, control_yaw_rad, 1.0, steering_reference_rad);

  overtake_planner::FrenetFrame frame;
  frame.setReference(reference);
  overtake_planner::CartesianTrackabilityInput input;
  input.ego.valid = true;
  input.ego.x = control_point.x;
  input.ego.y = control_point.y;
  input.ego.yaw = control_yaw_rad;
  input.ego.v = 1.0;
  input.active_lookahead_valid = true;
  input.active_lookahead_distance_m = production.lookahead_distance_m;
  input.nearest_source_index_valid = true;
  input.nearest_source_index = production.nearest_source_index;
  input.curvature_feedforward_valid = true;
  input.curvature_feedforward_steering_rad =
      production.curvature_feedforward_rad;
  input.steering_reference_valid = true;
  input.steering_reference_angle_rad = production.limiter_reference_rad;
  input.steering_command_dt_sec = 0.01;
  overtake_planner::CartesianTrackabilityConfig config;
  config.wheelbase_m = 1.087;
  config.steering_gain = 1.0;
  config.max_steering_angle_rad = 0.64;
  config.max_steering_rate_radps = parameter.hard_rate_limit_radps;
  config.max_yaw_tangent_error_rad = 0.1;
  config.pure_pursuit_required_arc_m = production.lookahead_distance_m;
  config.target_d_m = 0.0;
  config.target_d_deadline_arc_m = 2.0;
  const auto evaluated =
      overtake_planner::CartesianTrackabilityEvaluator(frame).evaluate(
          candidate, input, config);

  ASSERT_TRUE(evaluated.valid) << evaluated.reason;
  EXPECT_GT(std::abs(production.curvature_feedforward_rad), 1.0e-4);
  EXPECT_DOUBLE_EQ(production.limiter_reference_rad, steering_reference_rad);
  EXPECT_EQ(evaluated.pure_pursuit_target_source_index,
            production.selected_lookahead_source_index);
  EXPECT_NEAR(evaluated.pure_pursuit_lookahead_distance_m,
              production.lookahead_distance_m, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_geometric_steering_angle_rad,
              production.geometric_steering_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.curvature_feedforward_steering_angle_rad,
              production.curvature_feedforward_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_raw_steering_angle_rad,
              production.raw_steering_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_requested_steering_angle_rad,
              production.requested_steering_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_bounded_steering_angle_rad,
              production.bounded_steering_rad, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_requested_steering_rate_radps,
              production.requested_rate_radps, 1.0e-12);
  EXPECT_NEAR(evaluated.pure_pursuit_bounded_steering_rate_radps,
              production.bounded_rate_radps, 1.0e-12);
  EXPECT_EQ(evaluated.pure_pursuit_angle_limited, production.angle_limited);
  EXPECT_EQ(evaluated.pure_pursuit_rate_limited, production.rate_limited);
  EXPECT_EQ(production.rate_limited, parameter.expect_rate_limited);
  node.reset();
  rclcpp::shutdown();
}

INSTANTIATE_TEST_SUITE_P(
    NominalAndRateClamped, CurvedPurePursuitExactGolden,
    ::testing::Values(CurvedPurePursuitGoldenParam{128.0, false},
                      CurvedPurePursuitGoldenParam{10.0, true}));

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

TEST(ControlCyclePlanStore, AdoptsTypedPlanAtNextCycleAndBindsPublishBundle) {
  simple_pure_pursuit::ControlCyclePlanStore store;
  builtin_interfaces::msg::Time now_stamp;
  now_stamp.sec = 100;

  auto plan_n_minus_1 =
      std::make_shared<multi_purpose_mpc_ros_msgs::msg::OvertakePlan>();
  plan_n_minus_1->header.stamp.sec = 10;
  plan_n_minus_1->plan_generation = 57U;
  ASSERT_TRUE(store.accept(plan_n_minus_1, now_stamp, 1.0));
  const auto cycle_n_minus_1 = store.beginControlCycle();
  ASSERT_NE(cycle_n_minus_1, nullptr);
  ASSERT_NE(cycle_n_minus_1->plan, nullptr);
  EXPECT_EQ(cycle_n_minus_1->plan->plan_generation, 57U);

  auto plan_n =
      std::make_shared<multi_purpose_mpc_ros_msgs::msg::OvertakePlan>();
  plan_n->header.stamp.sec = 11;
  plan_n->plan_generation = 58U;
  plan_n->aw2_identity_schema_version = 1U;
  plan_n->planner_instance_id = 2U;
  plan_n->race_arm_epoch = 3U;
  plan_n->attempt_id = 4U;
  plan_n->target_vehicle_id = "D2";
  plan_n->pass_direction = 1;
  plan_n->connector_transaction_id = 5U;
  plan_n->candidate_revision = 6U;
  ASSERT_TRUE(store.accept(plan_n, now_stamp, 2.0));

  const auto cycle_n = store.beginControlCycle();
  ASSERT_NE(cycle_n, nullptr);
  ASSERT_NE(cycle_n->plan, nullptr);
  ASSERT_EQ(cycle_n->plan->plan_generation, 58U);
  EXPECT_EQ(cycle_n->plan->header.stamp, plan_n->header.stamp);

  autoware_auto_control_msgs::msg::AckermannControlCommand command;
  command.stamp.sec = 12;
  command.stamp.nanosec = 34U;
  command.longitudinal.stamp = command.stamp;
  command.lateral.stamp = command.stamp;
  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus status;
  status.header.stamp = command.stamp;
  status.plan_generation = cycle_n->plan->plan_generation;
  const auto envelope = simple_pure_pursuit::makeControllerCommandEnvelopeV1(
      command, status, 99U, 7U, cycle_n->plan.get());

  EXPECT_EQ(command.stamp, status.header.stamp);
  EXPECT_EQ(command.stamp, envelope.header.stamp);
  EXPECT_EQ(command.stamp, envelope.command.stamp);
  EXPECT_EQ(status.plan_generation, 58U);
  EXPECT_EQ(envelope.plan_generation, 58U);
  EXPECT_EQ(envelope.plan_sample_key.plan_generation, 58U);
  EXPECT_EQ(envelope.plan_sample_key.plan_stamp, plan_n->header.stamp);
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
  EXPECT_EQ(envelope.candidate_content_sha256, plan.candidate_content_sha256);
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
     Schema2CarriesExactSlidingGeometryBeyondLegacyPrefix) {
  auto command = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteExecutionWitnessInput(command);
  input.base_trajectory.points.clear();
  for (std::size_t index = 0U; index < 150U; ++index) {
    input.base_trajectory.points.push_back(
        makePointWithSpeed(static_cast<double>(index) * 0.1, 0.0, 0.0, 1.0));
  }
  input.applied_trajectory = input.base_trajectory;
  for (auto &point : input.applied_trajectory.points) {
    point.pose.position.y = 0.2;
  }
  input.nearest_trajectory_index = 120U;
  input.control_pose = input.applied_trajectory.points[120U].pose;

  auto plan = makeExecutionTestPlan(command, input);
  plan.aw2_identity_schema_version = 1U;
  plan.planner_instance_id = 77U;
  plan.race_arm_epoch = 5U;
  plan.connector_transaction_id = 9U;
  plan.candidate_revision = 3U;
  plan.candidate_content_sha256.fill(0x44U);
  command.schema_version = 2U;
  command.plan_sample_key.race_arm_epoch = plan.race_arm_epoch;
  command.plan_sample_key.planner_instance_id = plan.planner_instance_id;
  command.plan_sample_key.attempt_id = plan.attempt_id;
  command.plan_sample_key.target_vehicle_id = plan.target_vehicle_id;
  command.plan_sample_key.pass_direction = plan.pass_direction;
  command.plan_sample_key.connector_transaction_id =
      plan.connector_transaction_id;
  command.plan_sample_key.plan_stamp = plan.header.stamp;
  command.plan_sample_key.plan_generation = plan.plan_generation;
  command.candidate_revision = plan.candidate_revision;
  command.candidate_content_sha256 = plan.candidate_content_sha256;
  auto exact = makeExactExecutionSnapshot(command, input, 120U, 125U);

  const auto envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV2(
      command, input, &plan, true, exact);
  EXPECT_EQ(envelope.schema_version, 2U);
  EXPECT_TRUE(envelope.witness.diagnostic_only);
  EXPECT_TRUE(envelope.witness.exact_snapshot_complete);
  EXPECT_EQ(envelope.witness.exact_snapshot_reason, "complete");
  EXPECT_EQ(envelope.witness.exact_geometry_first_source_index, 120U);
  EXPECT_EQ(envelope.witness.exact_geometry_last_source_index, 125U);
  EXPECT_EQ(envelope.witness.exact_geometry_original_point_count, 150U);
  EXPECT_EQ(envelope.witness.applied_trajectory.points.size(), 6U);
  EXPECT_FALSE(envelope.witness.authority_eligible);
}

TEST(ControllerExecutionEnvelope,
     Schema2RateClampedNonzeroWitnessValidatesEndToEndWithZeroWireRate) {
  auto command = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteExecutionWitnessInput(command);
  auto plan = makeExecutionTestPlan(command, input);
  plan.aw2_identity_schema_version = 1U;
  plan.planner_instance_id = 77U;
  plan.race_arm_epoch = 5U;
  plan.connector_transaction_id = 9U;
  plan.candidate_revision = 3U;
  plan.candidate_content_sha256.fill(0x44U);
  command.schema_version = 2U;
  command.plan_sample_key.race_arm_epoch = plan.race_arm_epoch;
  command.plan_sample_key.planner_instance_id = plan.planner_instance_id;
  command.plan_sample_key.attempt_id = plan.attempt_id;
  command.plan_sample_key.target_vehicle_id = plan.target_vehicle_id;
  command.plan_sample_key.pass_direction = plan.pass_direction;
  command.plan_sample_key.connector_transaction_id =
      plan.connector_transaction_id;
  command.plan_sample_key.plan_stamp = plan.header.stamp;
  command.plan_sample_key.plan_generation = plan.plan_generation;
  command.candidate_revision = plan.candidate_revision;
  command.candidate_content_sha256 = plan.candidate_content_sha256;

  // Production publishes zero in the wire rotation_rate field. The nonzero
  // limiter rate is represented by the bounded angle delta in the witness.
  command.command.lateral.steering_tire_angle = 0.18F;
  command.command.lateral.steering_tire_rotation_rate = 0.0F;
  auto exact = makeExactExecutionSnapshot(command, input, 0U, 1U);
  exact.geometric_steering_tire_angle_rad = 0.55;
  exact.curvature_feedforward_steering_rad = 0.05;
  exact.raw_steering_tire_angle_rad = 0.60;
  exact.steering_output_gain = 1.0;
  exact.requested_output_steering_tire_angle_rad = 0.60;
  exact.limiter_reference_steering_rad = 0.10;
  exact.command_dt_sec = 0.01;
  exact.hard_steering_angle_limit_rad = 0.64;
  exact.hard_steering_rate_limit_radps = 8.0;
  exact.requested_steering_tire_rotation_rate_radps = 50.0;
  exact.bounded_steering_tire_angle_rad = 0.18;
  exact.bounded_steering_tire_rotation_rate_radps = 8.0;
  exact.steering_angle_limited = false;
  exact.steering_rate_limited = true;

  const auto envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV2(
      command, input, &plan, true, exact);
  ASSERT_TRUE(envelope.witness.exact_snapshot_complete)
      << envelope.witness.exact_snapshot_reason;
  EXPECT_DOUBLE_EQ(
      envelope.command_envelope.command.lateral.steering_tire_rotation_rate,
      0.0);
  EXPECT_DOUBLE_EQ(
      envelope.witness.bounded_exact_steering_tire_rotation_rate_radps, 8.0);

  overtake_planner::PurePursuitCandidateBinding expected;
  expected.race_arm_epoch = plan.race_arm_epoch;
  expected.planner_instance_id = plan.planner_instance_id;
  expected.attempt_id = plan.attempt_id;
  expected.target_vehicle_id = plan.target_vehicle_id;
  expected.pass_direction = plan.pass_direction;
  expected.connector_transaction_id = plan.connector_transaction_id;
  expected.plan_stamp_sec = 12.0;
  expected.plan_generation = plan.plan_generation;
  expected.candidate_revision = plan.candidate_revision;
  expected.candidate_content_sha256 = plan.candidate_content_sha256;
  const auto validated = overtake_planner::validatePurePursuitExactSnapshotV2(
      envelope, expected, 12.02);
  ASSERT_TRUE(validated.valid) << validated.reason;
  EXPECT_NEAR(validated.requested_steering_rate_radps, 50.0, 1.0e-12);
  EXPECT_NEAR(validated.bounded_steering_rate_radps, 8.0, 1.0e-12);
  EXPECT_TRUE(validated.steering_rate_limited);
}

TEST(ControllerExecutionEnvelope,
     Schema2RejectsMutatedSourceAndOutOfRangeLease) {
  auto command = makeExecutionTestCommandEnvelope();
  auto input = makeCompleteExecutionWitnessInput(command);
  auto plan = makeExecutionTestPlan(command, input);
  plan.aw2_identity_schema_version = 1U;
  plan.planner_instance_id = 77U;
  plan.race_arm_epoch = 5U;
  plan.connector_transaction_id = 9U;
  plan.candidate_revision = 3U;
  plan.candidate_content_sha256.fill(0x44U);
  command.schema_version = 2U;
  command.plan_sample_key.race_arm_epoch = plan.race_arm_epoch;
  command.plan_sample_key.planner_instance_id = plan.planner_instance_id;
  command.plan_sample_key.attempt_id = plan.attempt_id;
  command.plan_sample_key.target_vehicle_id = plan.target_vehicle_id;
  command.plan_sample_key.pass_direction = plan.pass_direction;
  command.plan_sample_key.connector_transaction_id =
      plan.connector_transaction_id;
  command.plan_sample_key.plan_stamp = plan.header.stamp;
  command.plan_sample_key.plan_generation = plan.plan_generation;
  command.candidate_revision = plan.candidate_revision;
  command.candidate_content_sha256 = plan.candidate_content_sha256;
  auto exact = makeExactExecutionSnapshot(command, input, 0U, 1U);

  exact.base_source_key.source_stamp.nanosec += 1U;
  auto envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV2(
      command, input, &plan, true, exact);
  EXPECT_FALSE(envelope.witness.exact_snapshot_complete);
  EXPECT_EQ(envelope.witness.exact_snapshot_reason,
            "base_source_binding_invalid");

  exact = makeExactExecutionSnapshot(command, input, 0U, 1U);
  exact.diagnostic_lease_duration_sec = 0.2;
  envelope = simple_pure_pursuit::makeControllerExecutionEnvelopeV2(
      command, input, &plan, true, exact);
  EXPECT_FALSE(envelope.witness.exact_snapshot_complete);
  EXPECT_EQ(envelope.witness.exact_snapshot_reason, "exact_scalar_invalid");
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

TEST(FreeRunLiveExact, UsedIntervalIsBoundedAndCarriesAllReadIndices) {
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 20;
  for (std::size_t index = 0U; index < 120U; ++index) {
    auto point =
        makePointWithSpeed(static_cast<double>(index) * 0.1, 0.0, 0.0, 2.0);
    point.time_from_start.nanosec =
        static_cast<std::uint32_t>(index * 1000000U);
    trajectory.points.push_back(point);
  }

  const auto geometry = simple_pure_pursuit::buildControllerGeometryV1(
      trajectory, 120U, 10U, 12U, 18U, 20U, 25U, false, 1.0);

  ASSERT_TRUE(geometry.bounded);
  ASSERT_TRUE(geometry.valid);
  EXPECT_EQ(geometry.geometry.first_source_index, 10U);
  EXPECT_EQ(geometry.geometry.last_source_index, 25U);
  EXPECT_EQ(geometry.geometry.points.size(), 16U);
  EXPECT_EQ(geometry.geometry.speed_cap_source_index, 12U);
  EXPECT_EQ(geometry.geometry.curvature_last_read_source_index, 18U);
  EXPECT_EQ(geometry.geometry.lookahead_selected_source_index, 20U);
  EXPECT_EQ(geometry.geometry.required_horizon_end_source_index, 25U);
  EXPECT_GE(geometry.geometry.total_arc_length_m, 1.0);
  EXPECT_TRUE(std::any_of(geometry.geometry.geometry_sha256.begin(),
                          geometry.geometry.geometry_sha256.end(),
                          [](std::uint8_t value) { return value != 0U; }));
}

TEST(FreeRunLiveExact, UsedIntervalRejectsMoreThanOneHundredPoints) {
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 20;
  for (std::size_t index = 0U; index < 120U; ++index) {
    trajectory.points.push_back(
        makePointWithSpeed(static_cast<double>(index) * 0.1, 0.0, 0.0, 2.0));
  }

  const auto geometry = simple_pure_pursuit::buildControllerGeometryV1(
      trajectory, 120U, 0U, 0U, 50U, 80U, 100U, false, 1.0);

  EXPECT_FALSE(geometry.bounded);
  EXPECT_FALSE(geometry.valid);
  EXPECT_TRUE(geometry.geometry.points.empty());
}

TEST(FreeRunLiveExact, CommandDigestBindsPayloadAndStamp) {
  auto command = makeExecutionTestCommandEnvelope().command;
  const auto baseline =
      simple_pure_pursuit::canonicalControllerCommandDigestV1(command);

  auto changed_payload = command;
  changed_payload.lateral.steering_tire_angle += 0.001F;
  EXPECT_NE(baseline, simple_pure_pursuit::canonicalControllerCommandDigestV1(
                          changed_payload));

  auto changed_stamp = command;
  ++changed_stamp.stamp.nanosec;
  EXPECT_NE(baseline, simple_pure_pursuit::canonicalControllerCommandDigestV1(
                          changed_stamp));

  command.longitudinal.speed = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(simple_pure_pursuit::canonicalControllerCommandDigestV1(command),
            simple_pure_pursuit::Aw2Sha256Digest{});
}

TEST(FreeRunLiveExact, AckCanonicalV2IsDeterministicAndBindsFields) {
  multi_purpose_mpc_ros_msgs::msg::FreeRunExecutionAck ack;
  ack.schema_version = ack.SCHEMA_V2;
  ack.ack_eligible = true;
  ack.evidence_state = ack.EVIDENCE_COMPLETE;
  ack.evidence_reason = "complete";
  ack.header.frame_id = "base_link";
  ack.header.stamp.sec = 10;
  ack.plan_key.canonical_algorithm_version = 1U;
  ack.plan_key.race_arm_epoch = 1U;
  ack.plan_key.planner_instance_id = 101U;
  ack.plan_key.plan_generation = 9U;
  ack.plan_key.plan_stamp = ack.header.stamp;
  ack.plan_key.canonical_plan_payload_sha256.fill(0x11U);
  ack.source_key.canonical_algorithm_version = 1U;
  ack.source_key.baseline_instance_id = 201U;
  ack.source_key.controller_instance_id = 201U;
  ack.source_key.source_generation = 3U;
  ack.source_key.source_stamp = ack.header.stamp;
  ack.source_key.original_point_count = 10U;
  ack.source_key.baseline_reference_sha256.fill(0x22U);
  ack.source_key.controller_implementation_sha256.fill(0x33U);
  ack.source_key.controller_config_sha256.fill(0x44U);
  ack.controller_sequence = 7U;
  ack.controller_command_stamp = ack.header.stamp;
  ack.control_pose.orientation.w = 1.0;
  ack.control_pose_stamp = ack.header.stamp;
  for (auto *command :
       {&ack.raw_controller_command, &ack.output_controller_command}) {
    command->stamp = ack.header.stamp;
    command->lateral.stamp = ack.header.stamp;
    command->longitudinal.stamp = ack.header.stamp;
    command->longitudinal.speed = 2.0F;
    command->lateral.steering_tire_angle = 0.2F;
  }
  ack.raw_controller_command_sha256.fill(0x55U);
  ack.output_controller_command_sha256.fill(0x66U);
  ack.raw_steering_tire_angle_rad = 0.2;
  ack.output_steering_tire_angle_rad = 0.2;
  ack.hard_steering_tire_angle_limit_rad = 0.3665191429188092;
  ack.required_spatial_horizon_m = 0.5;
  ack.available_spatial_horizon_m = 1.0;
  auto &geometry = ack.base_geometry;
  geometry.frame_id = "map";
  geometry.source_stamp = ack.header.stamp;
  geometry.original_point_count = 10U;
  geometry.first_source_index = 0U;
  geometry.last_source_index = 1U;
  geometry.nearest_source_index = 0U;
  geometry.speed_cap_source_index = 0U;
  geometry.curvature_last_read_source_index = 1U;
  geometry.lookahead_selected_source_index = 1U;
  geometry.required_horizon_end_source_index = 1U;
  geometry.required_spatial_horizon_m = 0.5;
  geometry.total_arc_length_m = 1.0;
  for (double x_m : {0.0, 1.0}) {
    multi_purpose_mpc_ros_msgs::msg::CandidateExecutionPoint point;
    point.position_x_m = x_m;
    point.orientation_w = 1.0;
    point.longitudinal_velocity_mps = 2.0F;
    geometry.points.push_back(point);
  }
  geometry.geometry_sha256.fill(0x77U);
  geometry.start_point_sha256.fill(0x78U);
  geometry.end_point_sha256.fill(0x79U);
  geometry.full_source_digest_state = geometry.FULL_SOURCE_DIGEST_COMPLETE;
  geometry.full_source_sha256.fill(0x22U);
  ack.applied_geometry = geometry;

  const auto baseline =
      simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(ack);
  constexpr simple_pure_pursuit::Aw2Sha256Digest kGoldenDigest{
      0xc8U, 0x83U, 0x64U, 0xb6U, 0xe3U, 0x67U, 0x87U, 0x1eU,
      0xb4U, 0xfaU, 0x8fU, 0x73U, 0xa6U, 0x33U, 0x45U, 0x0aU,
      0x99U, 0xafU, 0x20U, 0x23U, 0xa3U, 0xa6U, 0x0dU, 0x7aU,
      0x62U, 0xaeU, 0xabU, 0x79U, 0xc0U, 0xe3U, 0x11U, 0xd4U};
  EXPECT_EQ(baseline, kGoldenDigest);
  for (std::size_t iteration = 0U; iteration < 100U; ++iteration) {
    EXPECT_EQ(baseline,
              simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(ack));
  }
  auto hard_limit_mutation = ack;
  hard_limit_mutation.hard_steering_tire_angle_limit_rad += 0.001;
  EXPECT_NE(baseline, simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(
                          hard_limit_mutation));
  auto pose_mutation = ack;
  pose_mutation.control_pose.position.y = 0.01;
  EXPECT_NE(baseline, simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(
                          pose_mutation));
  auto evidence_mutation = ack;
  evidence_mutation.evidence_reason = "mutated";
  EXPECT_NE(baseline, simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(
                          evidence_mutation));
  ack.ack_sha256 = baseline;
  EXPECT_EQ(baseline,
            simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(ack));

  auto v1 = ack;
  v1.schema_version = v1.SCHEMA_V1;
  EXPECT_EQ(simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(v1),
            simple_pure_pursuit::Aw2Sha256Digest{});

  auto max_points = ack;
  max_points.source_key.original_point_count = 100U;
  for (auto *max_geometry :
       {&max_points.base_geometry, &max_points.applied_geometry}) {
    max_geometry->original_point_count = 100U;
    max_geometry->last_source_index = 99U;
    max_geometry->curvature_last_read_source_index = 99U;
    max_geometry->lookahead_selected_source_index = 99U;
    max_geometry->required_horizon_end_source_index = 99U;
    max_geometry->total_arc_length_m = 99.0;
    max_geometry->points.clear();
    for (std::size_t index = 0U; index < 100U; ++index) {
      multi_purpose_mpc_ros_msgs::msg::CandidateExecutionPoint point;
      point.position_x_m = static_cast<double>(index);
      point.orientation_w = 1.0;
      point.longitudinal_velocity_mps = 2.0F;
      max_geometry->points.push_back(point);
    }
  }
  constexpr simple_pure_pursuit::Aw2Sha256Digest kMaxGoldenDigest{
      0x1cU, 0xc3U, 0xc4U, 0x7aU, 0xe9U, 0xccU, 0x83U, 0x61U,
      0xceU, 0xc7U, 0x85U, 0x5fU, 0x78U, 0xbdU, 0x54U, 0x6aU,
      0xedU, 0xc3U, 0xb5U, 0x06U, 0xafU, 0xc2U, 0xf3U, 0xf0U,
      0xa0U, 0x27U, 0x5bU, 0xf5U, 0x31U, 0x8dU, 0x71U, 0x1dU};
  const auto max_digest =
      simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(max_points);
  EXPECT_EQ(max_digest, kMaxGoldenDigest);
  for (std::size_t iteration = 0U; iteration < 100U; ++iteration) {
    EXPECT_EQ(
        max_digest,
        simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(max_points));
  }

  auto invalid_time = ack;
  invalid_time.control_pose_stamp.nanosec = 1000000000U;
  EXPECT_EQ(
      simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(invalid_time),
      simple_pure_pursuit::Aw2Sha256Digest{});
  auto nonfinite = ack;
  nonfinite.applied_geometry.points[1].position_x_m =
      std::numeric_limits<double>::infinity();
  EXPECT_EQ(
      simple_pure_pursuit::canonicalFreeRunExecutionAckDigestV2(nonfinite),
      simple_pure_pursuit::Aw2Sha256Digest{});
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
