#include "state_lattice_overtake_planner/reference_override_contract.hpp"

#include "simple_pure_pursuit/overtake_override_contract.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace sl = state_lattice_overtake_planner;

constexpr sl::WirePublicationPolicy kExactLivePolicy{true, true};

TEST(ReferenceOverrideContract,
     PublicationPreparationKeepsShadowWithoutExplicitAuthority) {
  sl::PlannerOutput output;
  output.spatial_profile_shadow_only = true;
  EXPECT_TRUE(sl::prepareWireOutputForPublication(output, false)
                  .spatial_profile_shadow_only);
}

TEST(ReferenceOverrideContract,
     LivePublicationCannotBypassMissingBaseAttestation) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = true;
  output.mode = sl::BehaviorMode::OVERTAKE_RIGHT;
  output.intent = sl::SolverHorizonIntent::MANEUVER_AUTHORIZED;
  output.speed_cap_mps = 2.0;
  output.lateral_offsets_m = {0.0, -0.4};
  output.speed_caps_mps = {2.0, 1.5};
  output.longitudinal_offsets_m = {0.0, 1.0};
  output.spatial_profile_shadow_only = true;
  const auto wire_output = sl::prepareWireOutputForPublication(output, false);
  EXPECT_TRUE(wire_output.spatial_profile_shadow_only);
  EXPECT_EQ(sl::makeWirePayload(wire_output, 1U, kExactLivePolicy).kind,
            sl::WireKind::SPEED_ONLY_V2);
  EXPECT_TRUE(output.spatial_profile_shadow_only);
}

TEST(ReferenceOverrideContract,
     V2BaseIdentityStillClearsOnlyWireCopyShadowMarker) {
  sl::PlannerOutput output;
  output.spatial_profile_shadow_only = true;
  const auto wire_output = sl::prepareWireOutputForPublication(output, true);
  EXPECT_FALSE(wire_output.spatial_profile_shadow_only);
  EXPECT_TRUE(output.spatial_profile_shadow_only);
}

TEST(ReferenceOverrideContract, BuildsV4InReceiverOrder) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = true;
  output.mode = sl::BehaviorMode::OVERTAKE_LEFT;
  output.intent = sl::SolverHorizonIntent::MANDATORY_AVOIDANCE;
  output.lateral_offsets_m = {0.0, 0.4};
  output.speed_caps_mps = {3.0, 2.0};
  output.longitudinal_offsets_m = {0.0, 1.25};
  const auto payload = sl::makeWirePayload(output, 42U, kExactLivePolicy);
  ASSERT_EQ(payload.kind, sl::WireKind::SPATIAL_LATERAL_AND_SPEED_V4);
  const std::vector<float> expected{1.0F, 4.0F, 2.0F,  0.0F, 0.4F,  3.0F,
                                    2.0F, 0.0F, 1.25F, 4.0F, 42.0F, 2.0F};
  EXPECT_EQ(payload.data, expected);
}

TEST(ReferenceOverrideContract, BuildsSafeStopV2AndExplicitInactive) {
  sl::PlannerOutput stop;
  stop.active = true;
  stop.mode = sl::BehaviorMode::SAFE_STOP;
  stop.speed_cap_mps = 0.2;
  EXPECT_EQ(sl::makeWirePayload(stop, 7U).data,
            (std::vector<float>{1.0F, 10.0F, 0.0F, 2.0F, 7.0F, 0.2F}));
  sl::PlannerOutput inactive;
  EXPECT_EQ(sl::makeWirePayload(inactive, 8U).data,
            (std::vector<float>{1.0F, 0.0F, 0.0F, 1.0F, 8.0F}));
}

TEST(ReferenceOverrideContract, GenerationChangesOnlyForSemanticContent) {
  sl::PlannerOutput output;
  output.active = true;
  output.mode = sl::BehaviorMode::SAFE_STOP;
  output.speed_cap_mps = 0.2;
  const auto original = sl::makeWirePayload(output, 1U);
  EXPECT_TRUE(sl::semanticallyEqual(original, sl::makeWirePayload(output, 2U)));
  output.speed_cap_mps = 0.3;
  EXPECT_FALSE(
      sl::semanticallyEqual(original, sl::makeWirePayload(output, 2U)));
  EXPECT_EQ(sl::nextGeneration(16777215U), 1U);
  EXPECT_EQ(sl::nextGeneration(41U), 42U);
}

TEST(ReferenceOverrideContract,
     ExactCartesianAlwaysAdvancesAuthorityGeneration) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = true;
  output.mode = sl::BehaviorMode::OVERTAKE_LEFT;
  output.intent = sl::SolverHorizonIntent::MANDATORY_AVOIDANCE;
  output.execution_geometry_kind = sl::ExecutionGeometryKind::EXACT_CARTESIAN;
  output.lateral_offsets_m = {0.0, 0.0};
  output.speed_caps_mps = {1.0, 1.0};
  output.longitudinal_offsets_m = {0.0, 2.0};
  const auto previous = sl::makeWirePayload(output, 9U, kExactLivePolicy);
  const auto prospective = sl::makeWirePayload(output, 9U, kExactLivePolicy);
  ASSERT_TRUE(sl::semanticallyEqual(previous, prospective));
  EXPECT_TRUE(sl::shouldAdvanceWireGeneration(output, previous, prospective));

  output.execution_geometry_kind = sl::ExecutionGeometryKind::LEGACY_OFFSETS;
  EXPECT_FALSE(sl::shouldAdvanceWireGeneration(output, previous, prospective));
}

TEST(ReferenceOverrideContract, GenerationWrapsAtFloatExactLimit) {
  EXPECT_EQ(sl::nextGeneration(16777214U), 16777215U);
  EXPECT_EQ(sl::nextGeneration(16777215U), 1U);
  EXPECT_EQ(sl::nextGeneration(0U), 1U);
}

TEST(ReferenceOverrideContract, UnsafeLateralFallsBackWithoutPublishingD) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = false;
  output.mode = sl::BehaviorMode::SAFE_STOP;
  output.speed_cap_mps = 0.2;
  output.lateral_offsets_m = {0.0};
  output.speed_caps_mps = {0.2};
  const auto payload = sl::makeWirePayload(output, 3U, kExactLivePolicy);
  EXPECT_EQ(payload.kind, sl::WireKind::SPEED_ONLY_V2);
  EXPECT_EQ(payload.data.size(), 6U);
}

TEST(ReferenceOverrideContract, EmergencySafeStopCannotPublishLateralV4) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = true;
  output.emergency_stop = true;
  output.mode = sl::BehaviorMode::SAFE_STOP;
  output.intent = sl::SolverHorizonIntent::NONE;
  output.speed_cap_mps = 0.2;
  output.lateral_offsets_m = {0.0, 0.1};
  output.speed_caps_mps = {0.2, 0.2};
  output.longitudinal_offsets_m = {0.0, 1.0};
  EXPECT_EQ(sl::makeWirePayload(output, 3U, kExactLivePolicy).kind,
            sl::WireKind::SPEED_ONLY_V2);
}

TEST(ReferenceOverrideContract, MissingSpatialBindingNeverFallsBackToV3) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = true;
  output.mode = sl::BehaviorMode::FOLLOW_BLOCKED;
  output.speed_cap_mps = 0.2;
  output.lateral_offsets_m = {0.0, 0.1};
  output.speed_caps_mps = {0.2, 0.2};
  output.spatial_profile_shadow_only = true;
  const auto payload = sl::makeWirePayload(output, 4U);
  EXPECT_EQ(payload.kind, sl::WireKind::SPEED_ONLY_V2);
  EXPECT_EQ(payload.data.size(), 6U);
}

TEST(ReferenceOverrideContract, LiveGateCannotDowngradeMissingSpatialToV3) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = true;
  output.mode = sl::BehaviorMode::OVERTAKE_LEFT;
  output.lateral_offsets_m = {0.0, 0.1};
  output.speed_caps_mps = {1.0, 1.0};
  EXPECT_EQ(sl::makeWirePayload(output, 5U, kExactLivePolicy).kind,
            sl::WireKind::INACTIVE);
}

TEST(ReferenceOverrideContract, ExactGenerationWithoutLiveGateIsSpeedOnly) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = true;
  output.mode = sl::BehaviorMode::OVERTAKE_LEFT;
  output.speed_cap_mps = 0.2;
  output.lateral_offsets_m = {0.0, 0.1};
  output.speed_caps_mps = {1.0, 1.0};
  output.longitudinal_offsets_m = {0.0, 1.0};
  EXPECT_EQ(sl::makeWirePayload(output, 5U, {true, false}).kind,
            sl::WireKind::SPEED_ONLY_V2);
}

TEST(ReferenceOverrideContract, LiveGateWithoutExactGenerationIsSpeedOnly) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = true;
  output.mode = sl::BehaviorMode::OVERTAKE_LEFT;
  output.speed_cap_mps = 0.2;
  output.lateral_offsets_m = {0.0, 0.1};
  output.speed_caps_mps = {1.0, 1.0};
  output.longitudinal_offsets_m = {0.0, 1.0};
  EXPECT_EQ(sl::makeWirePayload(output, 5U, {false, true}).kind,
            sl::WireKind::SPEED_ONLY_V2);
}

TEST(ReferenceOverrideContract,
     ShadowSpatialProfileCannotPublishLateralAuthority) {
  sl::PlannerOutput output;
  output.active = true;
  output.safe_lateral = true;
  output.mode = sl::BehaviorMode::FOLLOW_BLOCKED;
  output.speed_cap_mps = 0.2;
  output.lateral_offsets_m = {0.0, 0.1};
  output.speed_caps_mps = {0.2, 0.2};
  output.longitudinal_offsets_m = {0.0, 1.0};
  output.spatial_profile_shadow_only = true;
  EXPECT_EQ(sl::makeWirePayload(output, 6U).kind, sl::WireKind::SPEED_ONLY_V2);
}

TEST(ReferenceOverrideContract,
     ExistingPurePursuitParserAcceptsAllGeneratedKinds) {
  sl::PlannerOutput lateral;
  lateral.active = true;
  lateral.safe_lateral = true;
  lateral.mode = sl::BehaviorMode::OVERTAKE_RIGHT;
  lateral.intent = sl::SolverHorizonIntent::MANDATORY_AVOIDANCE;
  lateral.lateral_offsets_m.assign(20U, -0.4);
  lateral.speed_caps_mps.assign(20U, 4.0);
  lateral.longitudinal_offsets_m.resize(20U);
  for (std::size_t i = 0U; i < lateral.longitudinal_offsets_m.size(); ++i) {
    lateral.longitudinal_offsets_m[i] = static_cast<double>(i);
  }
  const auto v4 = simple_pure_pursuit::parseOvertakeOverrideContract(
      sl::makeWirePayload(lateral, 9U, kExactLivePolicy).data);
  ASSERT_TRUE(v4.has_value());
  EXPECT_EQ(v4->kind, simple_pure_pursuit::OvertakeOverrideContractKind::
                          SPATIAL_LATERAL_AND_SPEED_V4);
  EXPECT_EQ(v4->lateral_offsets.size(), 20U);
  EXPECT_EQ(v4->longitudinal_offsets_m.size(), 20U);
  EXPECT_TRUE(v4->mandatory_lateral_avoidance);

  sl::PlannerOutput stop;
  stop.active = true;
  stop.mode = sl::BehaviorMode::SAFE_STOP;
  stop.speed_cap_mps = 0.2;
  const auto v2 = simple_pure_pursuit::parseOvertakeOverrideContract(
      sl::makeWirePayload(stop, 10U).data);
  ASSERT_TRUE(v2.has_value());
  EXPECT_EQ(v2->kind,
            simple_pure_pursuit::OvertakeOverrideContractKind::SPEED_ONLY_V2);

  const auto inactive = simple_pure_pursuit::parseOvertakeOverrideContract(
      sl::makeWirePayload(sl::PlannerOutput{}, 11U).data);
  ASSERT_TRUE(inactive.has_value());
  EXPECT_EQ(inactive->kind,
            simple_pure_pursuit::OvertakeOverrideContractKind::INACTIVE);
}

TEST(ReferenceOverrideContract, V4PocIdentityRequiresExactBaseTuple) {
  multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity identity;
  identity.plan_generation = 9U;
  identity.source_generation = 4U;
  identity.source_stamp.sec = 12;
  identity.source_stamp.nanosec = 34U;
  identity.frame_id = "map";

  EXPECT_TRUE(simple_pure_pursuit::v4PocIdentityMatches(
      identity, 9U, "map", identity.source_stamp, 4U));
  EXPECT_FALSE(simple_pure_pursuit::v4PocIdentityMatches(
      identity, 8U, "map", identity.source_stamp, 4U));
  EXPECT_FALSE(simple_pure_pursuit::v4PocIdentityMatches(
      identity, 9U, "odom", identity.source_stamp, 4U));
  auto other_stamp = identity.source_stamp;
  ++other_stamp.nanosec;
  EXPECT_FALSE(simple_pure_pursuit::v4PocIdentityMatches(identity, 9U, "map",
                                                         other_stamp, 4U));
  EXPECT_FALSE(simple_pure_pursuit::v4PocIdentityMatches(
      identity, 9U, "map", identity.source_stamp, 5U));
}

TEST(ReferenceOverrideContract,
     DeadlineNeverReusesPreventiveRoleAndFallsBackToEmptyLateralStop) {
  sl::PlannerOutput previous;
  previous.active = true;
  previous.mode = sl::BehaviorMode::SIDE_BY_SIDE_KEEP;
  previous.intent = sl::SolverHorizonIntent::MANDATORY_AVOIDANCE;
  previous.safe_lateral = true;
  previous.preventive_side_role_active = true;
  previous.lateral_offsets_m = {0.0, 0.2};
  previous.speed_caps_mps = {0.8, 0.8};
  previous.longitudinal_offsets_m = {0.0, 1.0};
  EXPECT_FALSE(sl::deadlinePreviousOutputReusable(true, true, previous));
  previous.preventive_side_role_active = false;
  EXPECT_TRUE(sl::deadlinePreviousOutputReusable(true, true, previous));
  EXPECT_FALSE(sl::deadlinePreviousOutputReusable(false, true, previous));
  EXPECT_FALSE(sl::deadlinePreviousOutputReusable(true, false, previous));

  const auto fallback = sl::makePlanningDeadlineStop(0.2);
  EXPECT_TRUE(fallback.active);
  EXPECT_EQ(fallback.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(fallback.intent, sl::SolverHorizonIntent::NONE);
  EXPECT_TRUE(fallback.emergency_stop);
  EXPECT_FALSE(fallback.safe_lateral);
  EXPECT_TRUE(fallback.lateral_offsets_m.empty());
  EXPECT_TRUE(fallback.speed_caps_mps.empty());
  EXPECT_TRUE(fallback.longitudinal_offsets_m.empty());
  EXPECT_DOUBLE_EQ(fallback.speed_cap_mps, 0.2);
  EXPECT_EQ(fallback.reason, "planning_deadline");
  EXPECT_EQ(sl::makeWirePayload(fallback, 1U).kind,
            sl::WireKind::SPEED_ONLY_V2);
}

TEST(ReferenceOverrideContract,
     DeadlineDiscardClearsOnlyUncommittedPassClearanceBinding) {
  sl::PlanningCycleMetrics metrics;
  metrics.candidate_generation_ms = 1.25;
  metrics.output_horizon_call_count = 7U;
  metrics.pass_clearance_diagnostic.evaluated = true;
  metrics.pass_clearance_diagnostic.inputs_valid = true;
  metrics.pass_clearance_diagnostic.observed_predicate = true;
  metrics.pass_clearance_diagnostic.target_id[0] = 'd';
  metrics.pass_clearance_diagnostic.target_id[1] = '2';

  sl::discardUncommittedPassClearanceDiagnostic(&metrics);

  EXPECT_FALSE(metrics.pass_clearance_diagnostic.evaluated);
  EXPECT_FALSE(metrics.pass_clearance_diagnostic.inputs_valid);
  EXPECT_FALSE(metrics.pass_clearance_diagnostic.observed_predicate);
  EXPECT_TRUE(
      std::string(metrics.pass_clearance_diagnostic.target_id.data()).empty());
  EXPECT_DOUBLE_EQ(metrics.candidate_generation_ms, 1.25);
  EXPECT_EQ(metrics.output_horizon_call_count, 7U);
  EXPECT_NO_THROW(sl::discardUncommittedPassClearanceDiagnostic(nullptr));
}
