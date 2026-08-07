#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <gtest/gtest.h>

#include <cstddef>

#define main aw2_shadow_supervisor_test_main
#include "../src/controller_applied_shadow_supervisor.cpp"
#undef main

namespace simple_pure_pursuit::aw2_shadow {
namespace {

FixedTrajectoryPoint makePoint(std::size_t source_index) {
  FixedTrajectoryPoint point{};
  point.position_x_m = static_cast<double>(source_index);
  point.orientation_w = 1.0;
  point.longitudinal_velocity_mps = 1.0F;
  return point;
}

FixedGeometry makeIntervalGeometry() {
  FixedGeometry geometry{};
  EXPECT_TRUE(copyFixedString("map", geometry.frame_id));
  geometry.source_stamp = FixedTime{12, 34U};
  geometry.original_point_count = 1000U;
  geometry.first_source_index = 400U;
  geometry.last_source_index = 439U;
  geometry.nearest_source_index = 400U;
  geometry.speed_cap_source_index = 405U;
  geometry.curvature_last_read_source_index = 430U;
  geometry.lookahead_selected_source_index = 420U;
  geometry.required_horizon_end_source_index = 439U;
  geometry.point_count = 40U;
  for (std::size_t index = 0U; index < geometry.point_count; ++index) {
    geometry.points[index] = makePoint(400U + index);
  }
  return geometry;
}

FixedSnapshot makeSnapshot() {
  FixedSnapshot snapshot{};
  snapshot.kind = SnapshotKind::kSample;
  snapshot.record_stamp = FixedTime{12, 34U};
  EXPECT_TRUE(copyFixedString("map", snapshot.record_frame_id));
  snapshot.controller_sample_key.race_arm_epoch = 5U;
  snapshot.controller_sample_key.planner_instance_id = 77U;
  snapshot.controller_sample_key.attempt_id = 8U;
  EXPECT_TRUE(
      copyFixedString("D2", snapshot.controller_sample_key.target_vehicle_id));
  snapshot.controller_sample_key.pass_direction = -1;
  snapshot.controller_sample_key.connector_transaction_id = 9U;
  snapshot.controller_sample_key.plan_stamp = FixedTime{12, 34U};
  snapshot.controller_sample_key.plan_generation = 42U;
  snapshot.controller_sample_key.controller_role = kControllerRolePrimary;
  snapshot.controller_sample_key.controller_instance_id = 99U;
  snapshot.controller_sample_key.controller_sequence = 7U;
  snapshot.controller_sample_key.controller_command_stamp = FixedTime{12, 34U};
  snapshot.candidate_revision = 3U;
  snapshot.candidate_content_sha256.fill(0x33U);
  snapshot.typed_plan_present = true;
  snapshot.typed_plan_fresh = true;
  snapshot.typed_plan_trajectory_authorized = true;
  snapshot.typed_plan_identity_schema_version = 1U;
  snapshot.geometry_relation = multi_purpose_mpc_ros_msgs::msg::
      ControllerAppliedEnvelope::GEOMETRY_RELATION_DIRECT_APPLIED;
  snapshot.base_geometry = makeIntervalGeometry();
  snapshot.applied_geometry = makeIntervalGeometry();
  snapshot.nearest_trajectory_index = 400U;
  snapshot.speed_cap_trajectory_index = 405U;
  snapshot.curvature_last_read_trajectory_index = 430U;
  snapshot.lookahead_selected_trajectory_index = 420U;
  snapshot.required_horizon_end_trajectory_index = 439U;
  snapshot.control_pose.orientation_w = 1.0;
  snapshot.control_pose_stamp = FixedTime{12, 34U};
  snapshot.raw_command.command_stamp = FixedTime{12, 34U};
  snapshot.raw_command.lateral_stamp = FixedTime{12, 34U};
  snapshot.raw_command.longitudinal_stamp = FixedTime{12, 34U};
  snapshot.output_command = snapshot.raw_command;
  snapshot.raw_steering_tire_angle_rad = 0.0;
  snapshot.output_steering_tire_angle_rad = 0.0;
  snapshot.raw_steering_tire_rotation_rate_radps = 0.0;
  snapshot.output_steering_tire_rotation_rate_radps = 0.0;
  snapshot.available_spatial_horizon_m = 39.0;
  snapshot.required_spatial_horizon_m = 39.0;
  snapshot.rollout_state = multi_purpose_mpc_ros_msgs::msg::
      ControllerAppliedEnvelope::ROLLOUT_UNAVAILABLE;
  return snapshot;
}

TEST(Aw2ShadowSupervisorConversion,
     PreservesAbsoluteIntervalProvenanceAcrossFixedSnapshot) {
  auto snapshot = makeSnapshot();
  SharedDataRegion data{};
  data.producer_build_digest.fill(0x11U);
  data.producer_config_digest.fill(0x22U);
  simple_pure_pursuit::ControllerAppliedBindingTracker tracker;

  const auto first = buildEnvelope(snapshot, data, tracker);
  const auto second = buildEnvelope(snapshot, data, tracker);

  EXPECT_EQ(first.schema_version, first.SCHEMA_V1);
  EXPECT_FALSE(first.authority_eligible);
  EXPECT_EQ(first.applied_geometry.original_point_count, 1000U);
  EXPECT_EQ(first.applied_geometry.first_source_index, 400U);
  EXPECT_EQ(first.applied_geometry.last_source_index, 439U);
  ASSERT_EQ(first.applied_geometry.points.size(), 40U);
  EXPECT_DOUBLE_EQ(first.applied_geometry.points.front().position_x_m, 400.0);
  EXPECT_DOUBLE_EQ(first.applied_geometry.points.back().position_x_m, 439.0);
  EXPECT_EQ(first.applied_geometry.full_source_digest_state,
            first.applied_geometry.FULL_SOURCE_DIGEST_INDETERMINATE);
  EXPECT_EQ(first.applied_geometry.geometry_sha256,
            second.applied_geometry.geometry_sha256);
}

} // namespace
} // namespace simple_pure_pursuit::aw2_shadow
