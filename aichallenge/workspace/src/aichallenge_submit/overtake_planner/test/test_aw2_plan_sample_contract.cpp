#include "overtake_planner/aw2_plan_sample_contract.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using overtake_planner::aw2::BindingObservation;
using overtake_planner::aw2::CandidateBinding;
using overtake_planner::aw2::CandidateContent;
using overtake_planner::aw2::CandidateExecutionRecord;
using overtake_planner::aw2::CandidateSourceKind;
using overtake_planner::aw2::ConnectorTransactionSequencer;
using overtake_planner::aw2::DeliveryObservation;
using overtake_planner::aw2::DeliveryRecordTracker;
using overtake_planner::aw2::Float32LayoutDimension;
using overtake_planner::aw2::Float32SourceWire;
using overtake_planner::aw2::GeometryRequirement;
using overtake_planner::aw2::SafetyEvaluationResult;
using overtake_planner::aw2::SameGenerationBindingTracker;
using overtake_planner::aw2::V2CanonicalSource;
using overtake_planner::aw2::ValidationError;

CandidateContent validContent() {
  CandidateContent content;
  content.key.transaction.race_arm_epoch = 3U;
  content.key.transaction.planner_instance_id = 7U;
  content.key.transaction.attempt_id = 11U;
  content.key.transaction.target_vehicle_id = "D2";
  content.key.transaction.pass_direction = -1;
  content.key.transaction.connector_transaction_id = 13U;
  content.key.plan_stamp_sec = 17;
  content.key.plan_stamp_nanosec = 19U;
  content.key.plan_generation = 23U;
  content.candidate_revision = 23U;
  content.frame_id = "map";
  content.source_kind = CandidateSourceKind::LEGACY_REFERENCE_OVERRIDE;
  content.geometry_point_count = 2U;
  content.source_wire = {0x00U, 0x01U, 0x7fU, 0x80U, 0xffU};
  return content;
}

CandidateExecutionRecord validExecutionRecord() {
  CandidateExecutionRecord record;
  record.key = validContent().key;
  record.candidate_revision = 23U;
  record.candidate_content_sha256.fill(0x5aU);
  record.plan_frame_id = "map";
  record.phase = 2U;
  record.authorization_state =
      overtake_planner::aw2::AuthorizationState::AUTHORIZED;
  record.geometry_requirement = GeometryRequirement::REQUIRED;
  record.candidate_type = 2U;
  record.trajectory_authorized_legacy = true;
  record.lateral_maneuver_required_legacy = true;
  record.published_pass_direction = -1;
  record.typed_trajectory_present = true;
  overtake_planner::aw2::CandidateExecutionPoint point;
  point.position_x_m = 1.0;
  point.position_y_m = 2.0;
  point.orientation_z = 0.25;
  point.orientation_w = 0.9682458365518543;
  point.longitudinal_velocity_mps = 3.0F;
  record.geometry_points.push_back(point);
  record.source_kind = CandidateSourceKind::LEGACY_REFERENCE_OVERRIDE;
  record.source_generation = 23U;
  record.canonical_source_wire = {1U, 2U, 3U, 4U};
  record.source_original_size_bytes = 4U;
  record.constraint_stamp_sec = 17;
  record.constraint_stamp_nanosec = 19U;
  record.constraint_frame_id = "map";
  record.constraint_generation = 29U;
  record.constraint_plan_generation = 23U;
  record.constraint_valid = true;
  record.constraint_stop_requested = false;
  record.constraint_release_authorized = true;
  record.constraint_speed_limit_mps = 8.0F;
  record.constraint_required_brake_decel_mps2 = 1.0F;
  record.constraint_reason = "ready";
  record.required_controller_spatial_horizon_m = 3.5;
  record.planned_target_d_m = -1.2;
  record.committed_target_d_m = -1.2;
  record.safety_snapshot_id = 31U;
  record.safety_evaluation_result = SafetyEvaluationResult::PASSED;
  record.safety_evaluation_reason = "passed";
  return record;
}

V2CanonicalSource validV2CanonicalSource() {
  V2CanonicalSource source;
  source.candidate_type = 2U;
  source.t = {0.0, 0.1};
  source.longitudinal_offsets_m = {0.0, 0.3};
  source.s = {1.0, 1.3};
  source.d = {0.0, -0.2};
  source.x = {2.0, 2.3};
  source.y = {3.0, 3.1};
  source.yaw = {0.1, 0.2};
  source.longitudinal_initial_measured_speed_mps = 2.0;
  source.predicted_speed_mps = {2.0, 2.1};
  source.v_ref = {2.0, 2.1};
  source.safety_evaluated = true;
  source.feasible = true;
  source.pass_target_corridor_valid = true;
  source.controller_tracking_profile_valid = true;
  source.desired_path_trackable = true;
  source.pure_pursuit_command_trackable = true;
  source.moving_target_relatively_reachable = true;
  source.planned_target_d_m = -1.0;
  source.committed_attack_follow_target_d_m = -1.0;
  source.required_controller_spatial_horizon_m = 3.5;
  source.controller_spatial_horizon_proof_valid = true;
  source.score = 1.0;
  source.min_safety_margin = 0.4;
  source.cbf_slack = 0.0;
  source.active_safety_constraint_count = 2;
  source.longitudinal_profile_valid = true;
  source.assumed_brake_decel_mps2 = 1.0;
  source.response_delay_sec = 0.1;
  source.required_brake_distance_m = 2.0;
  source.available_brake_distance_m = 3.0;
  source.reject_reason = "passed";
  return source;
}

std::string digestHex(const std::array<std::uint8_t, 32U> &digest) {
  std::ostringstream stream;
  stream << std::hex;
  for (const auto byte : digest) {
    stream.width(2);
    stream.fill('0');
    stream << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

TEST(Aw2ConnectorTransactionSequencer, IsStableAndMonotonicWithinRaceEpoch) {
  ConnectorTransactionSequencer sequencer;
  ASSERT_TRUE(sequencer.resetForRaceEpoch(4U));

  EXPECT_EQ(sequencer.update(4U, 0U, "", 0, false), std::nullopt);
  EXPECT_EQ(sequencer.update(4U, 8U, "D2", -1, true), 1U);
  EXPECT_EQ(sequencer.update(4U, 8U, "D2", -1, true), 1U);
  EXPECT_EQ(sequencer.update(4U, 8U, "D2", -1, false), std::nullopt);
  EXPECT_EQ(sequencer.update(4U, 9U, "D3", 1, true), 2U);
}

TEST(Aw2ConnectorTransactionSequencer, ResetsOnlyForNewRaceEpoch) {
  ConnectorTransactionSequencer sequencer;
  ASSERT_TRUE(sequencer.resetForRaceEpoch(4U));
  ASSERT_EQ(sequencer.update(4U, 8U, "D2", -1, true), 1U);
  EXPECT_EQ(sequencer.update(3U, 8U, "D2", -1, true), std::nullopt);
  EXPECT_FALSE(sequencer.resetForRaceEpoch(4U));
  EXPECT_FALSE(sequencer.resetForRaceEpoch(3U));

  ASSERT_TRUE(sequencer.resetForRaceEpoch(5U));
  EXPECT_EQ(sequencer.update(5U, 8U, "D2", -1, true), 1U);
}

TEST(Aw2ConnectorTransactionSequencer, WrapFailsClosedUntilNewRaceEpoch) {
  ConnectorTransactionSequencer sequencer(
      std::numeric_limits<std::uint64_t>::max());

  EXPECT_EQ(sequencer.update(1U, 8U, "D2", -1, true), std::nullopt);
  EXPECT_TRUE(sequencer.exhausted());
  EXPECT_EQ(sequencer.update(1U, 9U, "D3", 1, true), std::nullopt);

  ASSERT_TRUE(sequencer.resetForRaceEpoch(5U));
  EXPECT_EQ(sequencer.update(5U, 9U, "D3", 1, true), 1U);
}

TEST(Aw2PlanSampleContract, CanonicalBytesAndSha256MatchGolden) {
  const auto canonical =
      overtake_planner::aw2::canonicalizeCandidateContentV1(validContent());
  ASSERT_TRUE(canonical.valid());
  EXPECT_EQ(canonical.bytes.size(), 80U);
  const std::array<std::uint8_t, 32U> expected{
      0x75U, 0xa4U, 0x99U, 0x0dU, 0x70U, 0xc9U, 0x9dU, 0x8bU,
      0x1dU, 0x3cU, 0xc8U, 0xf0U, 0x50U, 0x19U, 0x5cU, 0xdaU,
      0xf5U, 0xf1U, 0x32U, 0xe5U, 0xacU, 0x80U, 0x5cU, 0x8aU,
      0x28U, 0x89U, 0x11U, 0xe5U, 0x11U, 0x89U, 0xcdU, 0x84U};
  EXPECT_EQ(canonical.sha256, expected);
}

TEST(Aw2PlanSampleContract, Sha256MatchesPublishedAbcVector) {
  const std::vector<std::uint8_t> abc{'a', 'b', 'c'};
  const std::array<std::uint8_t, 32U> expected{
      0xbaU, 0x78U, 0x16U, 0xbfU, 0x8fU, 0x01U, 0xcfU, 0xeaU,
      0x41U, 0x41U, 0x40U, 0xdeU, 0x5dU, 0xaeU, 0x22U, 0x23U,
      0xb0U, 0x03U, 0x61U, 0xa3U, 0x96U, 0x17U, 0x7aU, 0x9cU,
      0xb4U, 0x10U, 0xffU, 0x61U, 0xf2U, 0x00U, 0x15U, 0xadU};
  EXPECT_EQ(overtake_planner::aw2::sha256(abc), expected);
}

TEST(Aw2PlanSampleContract, Float32WireRejectsNonFiniteAndOversize) {
  const auto wire =
      overtake_planner::aw2::canonicalizeFloat32SourceWire({1.0F, -0.0F});
  ASSERT_TRUE(wire.has_value());
  EXPECT_EQ(wire->size(), 42U);

  EXPECT_FALSE(overtake_planner::aw2::canonicalizeFloat32SourceWire(
                   {std::numeric_limits<float>::quiet_NaN()})
                   .has_value());
  EXPECT_FALSE(overtake_planner::aw2::canonicalizeFloat32SourceWire(
                   {std::numeric_limits<float>::infinity()})
                   .has_value());
  EXPECT_FALSE(overtake_planner::aw2::canonicalizeFloat32SourceWire(
                   std::vector<float>(1023U, 0.0F))
                   .has_value());
}

TEST(Aw2PlanSampleContract, Float32WireBindsLayoutOffsetDimensionsAndAllData) {
  Float32SourceWire source;
  source.dimensions = {Float32LayoutDimension{"profile", 2U, 2U}};
  source.data_offset = 1U;
  source.values = {1.0F, -2.0F};
  const auto baseline =
      overtake_planner::aw2::canonicalizeFloat32SourceWire(source);
  ASSERT_TRUE(baseline.has_value());

  auto changed = source;
  changed.dimensions.front().label = "profiles";
  EXPECT_NE(overtake_planner::aw2::canonicalizeFloat32SourceWire(changed),
            baseline);
  changed = source;
  ++changed.dimensions.front().size;
  EXPECT_NE(overtake_planner::aw2::canonicalizeFloat32SourceWire(changed),
            baseline);
  changed = source;
  ++changed.dimensions.front().stride;
  EXPECT_NE(overtake_planner::aw2::canonicalizeFloat32SourceWire(changed),
            baseline);
  changed = source;
  ++changed.data_offset;
  EXPECT_NE(overtake_planner::aw2::canonicalizeFloat32SourceWire(changed),
            baseline);
  changed = source;
  changed.values.back() = -3.0F;
  EXPECT_NE(overtake_planner::aw2::canonicalizeFloat32SourceWire(changed),
            baseline);

  source.dimensions.resize(overtake_planner::aw2::kMaxSourceLayoutDimensions +
                           1U);
  EXPECT_FALSE(
      overtake_planner::aw2::canonicalizeFloat32SourceWire(source).has_value());
}

TEST(Aw2PlanSampleContract, CandidateDigestBindsEverySemanticIdentityField) {
  const auto baseline =
      overtake_planner::aw2::canonicalizeCandidateContentV1(validContent());
  ASSERT_TRUE(baseline.valid());

  auto expect_digest_change = [&baseline](CandidateContent changed) {
    const auto canonical =
        overtake_planner::aw2::canonicalizeCandidateContentV1(changed);
    ASSERT_TRUE(canonical.valid());
    EXPECT_NE(canonical.sha256, baseline.sha256);
  };

  auto changed = validContent();
  changed.frame_id = "odom";
  expect_digest_change(changed);
  changed = validContent();
  changed.key.transaction.target_vehicle_id = "D3";
  expect_digest_change(changed);
  changed = validContent();
  changed.key.transaction.pass_direction = 1;
  expect_digest_change(changed);
  changed = validContent();
  ++changed.key.transaction.connector_transaction_id;
  expect_digest_change(changed);
  changed = validContent();
  ++changed.geometry_point_count;
  expect_digest_change(changed);
  changed = validContent();
  changed.source_wire.back() ^= 0x01U;
  expect_digest_change(changed);
}

TEST(Aw2PlanSampleContract, RejectsBoundsBeforeDigest) {
  auto content = validContent();
  content.key.transaction.planner_instance_id = 0U;
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateContentV1(content).error,
      ValidationError::INVALID_IDENTITY);

  content = validContent();
  content.key.transaction.pass_direction = 0;
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateContentV1(content).error,
      ValidationError::INVALID_SIDE);

  content = validContent();
  content.frame_id.assign(overtake_planner::aw2::kMaxFrameIdBytes + 1U, 'f');
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateContentV1(content).error,
      ValidationError::FRAME_ID_LIMIT_EXCEEDED);

  content = validContent();
  content.key.transaction.target_vehicle_id.assign(
      overtake_planner::aw2::kMaxTargetIdBytes + 1U, 't');
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateContentV1(content).error,
      ValidationError::TARGET_ID_LIMIT_EXCEEDED);

  content = validContent();
  content.geometry_point_count = overtake_planner::aw2::kMaxGeometryPoints + 1U;
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateContentV1(content).error,
      ValidationError::GEOMETRY_LIMIT_EXCEEDED);

  content = validContent();
  content.source_wire.assign(overtake_planner::aw2::kMaxSourceWireBytes + 1U,
                             0U);
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateContentV1(content).error,
      ValidationError::SOURCE_WIRE_LIMIT_EXCEEDED);
}

TEST(Aw2PlanSampleContract, DuplicateIsConsistentButMutationFails) {
  const auto canonical =
      overtake_planner::aw2::canonicalizeCandidateContentV1(validContent());
  ASSERT_TRUE(canonical.valid());

  CandidateBinding binding;
  binding.race_arm_epoch = 3U;
  binding.planner_instance_id = 7U;
  binding.plan_generation = 23U;
  binding.candidate_revision = 23U;
  binding.candidate_content_sha256 = canonical.sha256;

  SameGenerationBindingTracker tracker;
  EXPECT_EQ(tracker.observe(binding), BindingObservation::ACCEPTED);
  EXPECT_EQ(tracker.observe(binding), BindingObservation::CONSISTENT);

  auto mutation = binding;
  mutation.candidate_content_sha256[0] ^= 0x01U;
  EXPECT_EQ(tracker.observe(mutation),
            BindingObservation::SAME_GENERATION_PAYLOAD_MUTATION);
  EXPECT_EQ(tracker.observe(binding),
            BindingObservation::SAME_GENERATION_PAYLOAD_MUTATION);

  auto revision_mutation = binding;
  ++revision_mutation.candidate_revision;
  EXPECT_EQ(tracker.observe(revision_mutation),
            BindingObservation::SAME_GENERATION_PAYLOAD_MUTATION);

  auto next_generation = binding;
  ++next_generation.plan_generation;
  ++next_generation.candidate_revision;
  EXPECT_EQ(tracker.observe(next_generation), BindingObservation::ACCEPTED);
  EXPECT_EQ(tracker.observe(next_generation), BindingObservation::CONSISTENT);
}

TEST(Aw2PlanSampleContract, AtomicDeliveryDigestBindsEveryRecordLayer) {
  const auto baseline =
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(
          validExecutionRecord());
  ASSERT_TRUE(baseline.valid());

  const auto expect_digest_change = [&baseline](
                                        CandidateExecutionRecord changed) {
    const auto canonical =
        overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(changed);
    ASSERT_TRUE(canonical.valid());
    EXPECT_NE(canonical.plan_sample_record_sha256,
              baseline.plan_sample_record_sha256);
  };

  auto changed = validExecutionRecord();
  changed.geometry_points.front().front_wheel_angle_rad = 0.1F;
  expect_digest_change(changed);
  changed = validExecutionRecord();
  changed.constraint_stop_requested = true;
  expect_digest_change(changed);
  changed = validExecutionRecord();
  changed.canonical_source_wire.back() ^= 0x01U;
  expect_digest_change(changed);
  changed = validExecutionRecord();
  changed.safety_evaluation_reason = "different";
  expect_digest_change(changed);
  changed = validExecutionRecord();
  changed.trajectory_authorized_legacy = false;
  expect_digest_change(changed);
  changed = validExecutionRecord();
  changed.lateral_maneuver_required_legacy = false;
  expect_digest_change(changed);
  changed = validExecutionRecord();
  changed.geometry_requirement = GeometryRequirement::UNKNOWN;
  expect_digest_change(changed);
  changed = validExecutionRecord();
  ++changed.key.plan_stamp_nanosec;
  ++changed.constraint_stamp_nanosec;
  expect_digest_change(changed);
}

TEST(Aw2PlanSampleContract, AtomicDeliveryDigestsMatchGolden) {
  const auto canonical =
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(
          validExecutionRecord());
  ASSERT_TRUE(canonical.valid());
  EXPECT_EQ(digestHex(canonical.delivered_geometry_sha256),
            "ccbed2b9633456d200202b051df27e5672281fdba5f4bc7badc16c8acd15ccf8");
  EXPECT_EQ(digestHex(canonical.canonical_source_sha256),
            "9f64a747e1b97f131fabb6b447296c9b6f0201e79fb3c5356e6c77e89b6a806a");
  EXPECT_EQ(digestHex(canonical.plan_sample_record_sha256),
            "d2ff5e511bbeba2c39b22af77c8b9fc3a58e2f3caa8ffefb47d4d62f0aa6bbc3");
}

TEST(Aw2PlanSampleContract,
     EmptyTypedTrajectoryIsValidOnlyWhenGeometryIsNotRequired) {
  auto record = validExecutionRecord();
  record.typed_trajectory_present = false;
  record.geometry_points.clear();
  record.geometry_requirement = GeometryRequirement::NOT_REQUIRED;
  EXPECT_TRUE(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .valid());

  record.geometry_requirement = GeometryRequirement::REQUIRED;
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .error,
      ValidationError::GEOMETRY_LIMIT_EXCEEDED);

  record.geometry_requirement = GeometryRequirement::UNKNOWN;
  record.lateral_maneuver_required_legacy = true;
  EXPECT_TRUE(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .valid());
}

TEST(Aw2PlanSampleContract, AtomicDeliveryRejectsNonFiniteAndBoundViolations) {
  auto record = validExecutionRecord();
  record.geometry_points.front().position_x_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .valid());

  record = validExecutionRecord();
  record.geometry_points.resize(overtake_planner::aw2::kMaxGeometryPoints + 1U);
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .error,
      ValidationError::GEOMETRY_LIMIT_EXCEEDED);

  record = validExecutionRecord();
  record.constraint_plan_generation++;
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .error,
      ValidationError::INVALID_GENERATION);

  record = validExecutionRecord();
  record.geometry_points.front().orientation_w = 2.0;
  EXPECT_FALSE(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .valid());

  record = validExecutionRecord();
  record.phase = 255U;
  EXPECT_FALSE(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .valid());

  for (std::size_t field_index = 0U; field_index < 5U; ++field_index) {
    record = validExecutionRecord();
    const double invalid = field_index % 2U == 0U
                               ? std::numeric_limits<double>::infinity()
                               : -std::numeric_limits<double>::infinity();
    switch (field_index) {
    case 0U:
      record.constraint_speed_limit_mps = static_cast<float>(invalid);
      break;
    case 1U:
      record.constraint_required_brake_decel_mps2 = static_cast<float>(invalid);
      break;
    case 2U:
      record.required_controller_spatial_horizon_m = invalid;
      break;
    case 3U:
      record.planned_target_d_m = invalid;
      break;
    default:
      record.committed_target_d_m = invalid;
      break;
    }
    EXPECT_FALSE(
        overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
            .valid());
  }

  record = validExecutionRecord();
  record.geometry_points.front().orientation_x = 0.0;
  record.geometry_points.front().orientation_y = 0.0;
  record.geometry_points.front().orientation_z = 0.0;
  record.geometry_points.front().orientation_w = 0.0;
  EXPECT_FALSE(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .valid());
}

TEST(Aw2PlanSampleContract, DeliveryConflictIsSeparateFromSemanticMutation) {
  const auto record = validExecutionRecord();
  const auto canonical =
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record);
  ASSERT_TRUE(canonical.valid());
  DeliveryRecordTracker tracker;
  EXPECT_EQ(tracker.observe(record.key, canonical.plan_sample_record_sha256),
            DeliveryObservation::ACCEPTED);
  EXPECT_EQ(tracker.observe(record.key, canonical.plan_sample_record_sha256),
            DeliveryObservation::CONSISTENT_DUPLICATE);

  auto different = canonical.plan_sample_record_sha256;
  different[0] ^= 0x01U;
  EXPECT_EQ(tracker.observe(record.key, different),
            DeliveryObservation::CONFLICT);
  EXPECT_EQ(tracker.observe(record.key, canonical.plan_sample_record_sha256),
            DeliveryObservation::CONFLICT);
}

TEST(Aw2PlanSampleContract, EvictedDeliveryKeyCannotBeAcceptedAgain) {
  DeliveryRecordTracker tracker;
  const auto digest =
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(
          validExecutionRecord())
          .plan_sample_record_sha256;
  auto key = validExecutionRecord().key;
  const auto oldest_key = key;
  for (std::size_t index = 0U;
       index <= overtake_planner::aw2::kDeliveryConflictHistorySize; ++index) {
    key.plan_generation = 23U + static_cast<std::uint32_t>(index);
    key.plan_stamp_nanosec = 19U + static_cast<std::uint32_t>(index);
    EXPECT_EQ(tracker.observe(key, digest), DeliveryObservation::ACCEPTED);
  }

  EXPECT_EQ(tracker.observe(oldest_key, digest), DeliveryObservation::INVALID);
  key.plan_generation++;
  key.plan_stamp_nanosec++;
  EXPECT_EQ(tracker.observe(key, digest), DeliveryObservation::ACCEPTED);
}

TEST(Aw2PlanSampleContract, V2CanonicalSourceBindsAllSemanticVectorsAndFlags) {
  const auto source = validV2CanonicalSource();
  const auto baseline = overtake_planner::aw2::canonicalizeV2SourceWire(source);
  ASSERT_TRUE(baseline.has_value());

  auto changed = source;
  changed.predicted_speed_mps.back() = 2.2;
  EXPECT_NE(overtake_planner::aw2::canonicalizeV2SourceWire(changed), baseline);
  changed = source;
  changed.controller_spatial_horizon_proof_valid = false;
  EXPECT_NE(overtake_planner::aw2::canonicalizeV2SourceWire(changed), baseline);
}

TEST(Aw2PlanSampleContract,
     V2CanonicalSourceEncodesFieldSpecificSentinelsAsAbsent) {
  const auto source = validV2CanonicalSource();
  const auto baseline = overtake_planner::aw2::canonicalizeV2SourceWire(source);
  ASSERT_TRUE(baseline.has_value());

  auto expect_distinct_valid = [&baseline](V2CanonicalSource changed) {
    const auto canonical =
        overtake_planner::aw2::canonicalizeV2SourceWire(changed);
    ASSERT_TRUE(canonical.has_value());
    EXPECT_NE(canonical, baseline);
  };

  auto changed = source;
  changed.longitudinal_initial_measured_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  expect_distinct_valid(changed);
  changed = source;
  changed.planned_target_d_m = std::numeric_limits<double>::quiet_NaN();
  expect_distinct_valid(changed);
  changed = source;
  changed.committed_attack_follow_target_d_m =
      std::numeric_limits<double>::quiet_NaN();
  expect_distinct_valid(changed);
  changed = source;
  changed.min_safety_margin = std::numeric_limits<double>::infinity();
  expect_distinct_valid(changed);
  changed = source;
  changed.required_brake_distance_m = std::numeric_limits<double>::infinity();
  expect_distinct_valid(changed);
  changed = source;
  changed.available_brake_distance_m = std::numeric_limits<double>::infinity();
  expect_distinct_valid(changed);

  changed = source;
  changed.min_safety_margin = std::numeric_limits<double>::infinity();
  const auto absent = overtake_planner::aw2::canonicalizeV2SourceWire(changed);
  changed.min_safety_margin = 0.0;
  const auto finite_zero =
      overtake_planner::aw2::canonicalizeV2SourceWire(changed);
  ASSERT_TRUE(absent.has_value());
  ASSERT_TRUE(finite_zero.has_value());
  EXPECT_NE(absent, finite_zero);
}

TEST(Aw2PlanSampleContract, V2CanonicalSourcePresenceEncodingMatchesGolden) {
  const auto present =
      overtake_planner::aw2::canonicalizeV2SourceWire(validV2CanonicalSource());
  ASSERT_TRUE(present.has_value());
  EXPECT_EQ(digestHex(overtake_planner::aw2::sha256(present.value())),
            "d15df71c592694b5c7719f67cca2a2977042e5a844d0a0a7649cc3237eb03dc7");

  auto absent_source = validV2CanonicalSource();
  absent_source.longitudinal_initial_measured_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  absent_source.planned_target_d_m = std::numeric_limits<double>::quiet_NaN();
  absent_source.committed_attack_follow_target_d_m =
      std::numeric_limits<double>::quiet_NaN();
  absent_source.min_safety_margin = std::numeric_limits<double>::infinity();
  absent_source.required_brake_distance_m =
      std::numeric_limits<double>::infinity();
  absent_source.available_brake_distance_m =
      std::numeric_limits<double>::infinity();
  const auto absent =
      overtake_planner::aw2::canonicalizeV2SourceWire(absent_source);
  ASSERT_TRUE(absent.has_value());
  EXPECT_EQ(digestHex(overtake_planner::aw2::sha256(absent.value())),
            "5c5f79852615fb60c77280be145a44a3bfda4c953cc81c7ea02cc125a6f9d121");
}

TEST(Aw2PlanSampleContract, V2CanonicalSourceDoesNotMutateRawSentinels) {
  auto source = validV2CanonicalSource();
  source.longitudinal_initial_measured_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  source.planned_target_d_m = std::numeric_limits<double>::quiet_NaN();
  source.committed_attack_follow_target_d_m =
      std::numeric_limits<double>::quiet_NaN();
  source.min_safety_margin = std::numeric_limits<double>::infinity();
  source.required_brake_distance_m = std::numeric_limits<double>::infinity();
  source.available_brake_distance_m = std::numeric_limits<double>::infinity();
  const auto expected_t = source.t;
  const auto expected_d = source.d;

  ASSERT_TRUE(
      overtake_planner::aw2::canonicalizeV2SourceWire(source).has_value());
  EXPECT_TRUE(std::isnan(source.longitudinal_initial_measured_speed_mps));
  EXPECT_TRUE(std::isnan(source.planned_target_d_m));
  EXPECT_TRUE(std::isnan(source.committed_attack_follow_target_d_m));
  EXPECT_EQ(source.min_safety_margin, std::numeric_limits<double>::infinity());
  EXPECT_EQ(source.required_brake_distance_m,
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(source.available_brake_distance_m,
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(source.t, expected_t);
  EXPECT_EQ(source.d, expected_d);
}

TEST(Aw2PlanSampleContract,
     V2OptionalDiagnosticsCanProduceACompleteAtomicRecord) {
  auto source = validV2CanonicalSource();
  source.min_safety_margin = std::numeric_limits<double>::infinity();
  source.required_brake_distance_m = std::numeric_limits<double>::infinity();
  source.available_brake_distance_m = std::numeric_limits<double>::infinity();
  const auto source_wire =
      overtake_planner::aw2::canonicalizeV2SourceWire(source);
  ASSERT_TRUE(source_wire.has_value());

  auto record = validExecutionRecord();
  record.source_kind = CandidateSourceKind::V2_TRAJECTORY;
  record.canonical_source_wire = source_wire.value();
  record.source_original_size_bytes =
      static_cast<std::uint32_t>(record.canonical_source_wire.size());
  EXPECT_TRUE(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .valid());
}

TEST(Aw2PlanSampleContract,
     V2AbsentTransactionTargetsRemainInvalidForAtomicRecord) {
  auto source = validV2CanonicalSource();
  source.planned_target_d_m = std::numeric_limits<double>::quiet_NaN();
  source.committed_attack_follow_target_d_m =
      std::numeric_limits<double>::quiet_NaN();
  const auto source_wire =
      overtake_planner::aw2::canonicalizeV2SourceWire(source);
  ASSERT_TRUE(source_wire.has_value());

  auto record = validExecutionRecord();
  record.source_kind = CandidateSourceKind::V2_TRAJECTORY;
  record.canonical_source_wire = source_wire.value();
  record.source_original_size_bytes =
      static_cast<std::uint32_t>(record.canonical_source_wire.size());
  record.planned_target_d_m = std::numeric_limits<double>::quiet_NaN();
  record.committed_target_d_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record)
          .error,
      ValidationError::CANONICAL_RECORD_LIMIT_EXCEEDED);
}

TEST(Aw2PlanSampleContract,
     V2CanonicalSourceRejectsWrongSentinelsAndRequiredNonFiniteValues) {
  const auto source = validV2CanonicalSource();
  const auto expect_invalid = [](V2CanonicalSource changed) {
    EXPECT_FALSE(
        overtake_planner::aw2::canonicalizeV2SourceWire(changed).has_value());
  };

  auto changed = source;
  changed.longitudinal_initial_measured_speed_mps =
      std::numeric_limits<double>::infinity();
  expect_invalid(changed);
  changed = source;
  changed.planned_target_d_m = std::numeric_limits<double>::infinity();
  expect_invalid(changed);
  changed = source;
  changed.committed_attack_follow_target_d_m =
      -std::numeric_limits<double>::infinity();
  expect_invalid(changed);
  changed = source;
  changed.min_safety_margin = std::numeric_limits<double>::quiet_NaN();
  expect_invalid(changed);
  changed = source;
  changed.required_brake_distance_m = -std::numeric_limits<double>::infinity();
  expect_invalid(changed);
  changed = source;
  changed.available_brake_distance_m = std::numeric_limits<double>::quiet_NaN();
  expect_invalid(changed);
  changed = source;
  changed.required_controller_spatial_horizon_m =
      std::numeric_limits<double>::quiet_NaN();
  expect_invalid(changed);
  changed = source;
  changed.score = std::numeric_limits<double>::infinity();
  expect_invalid(changed);
  changed = source;
  changed.cbf_slack = -std::numeric_limits<double>::infinity();
  expect_invalid(changed);
  changed = source;
  changed.predicted_speed_mps.back() = std::numeric_limits<double>::quiet_NaN();
  expect_invalid(changed);
}

TEST(Aw2PlanSampleContract, NewPlannerInstanceStartsIndependentBindingScope) {
  SameGenerationBindingTracker tracker;
  CandidateBinding first;
  first.race_arm_epoch = 3U;
  first.planner_instance_id = 7U;
  first.plan_generation = 23U;
  first.candidate_revision = 23U;
  first.candidate_content_sha256[0] = 1U;
  ASSERT_EQ(tracker.observe(first), BindingObservation::ACCEPTED);

  auto restarted = first;
  restarted.planner_instance_id = 8U;
  restarted.plan_generation = 1U;
  restarted.candidate_revision = 1U;
  restarted.candidate_content_sha256[0] = 2U;
  EXPECT_EQ(tracker.observe(restarted), BindingObservation::ACCEPTED);
}

TEST(Aw2PlanSampleContract, GenerationRegressionFails) {
  SameGenerationBindingTracker tracker;
  CandidateBinding first;
  first.race_arm_epoch = 3U;
  first.planner_instance_id = 7U;
  first.plan_generation = 23U;
  first.candidate_revision = 23U;
  first.candidate_content_sha256[0] = 1U;
  ASSERT_EQ(tracker.observe(first), BindingObservation::ACCEPTED);

  auto regression = first;
  regression.plan_generation = 22U;
  regression.candidate_revision = 22U;
  EXPECT_EQ(tracker.observe(regression),
            BindingObservation::GENERATION_REGRESSION);
}

TEST(Aw2PlanSampleContract, RaceArmEpochWrapFailsClosed) {
  EXPECT_EQ(overtake_planner::aw2::nextRaceArmEpoch(0U), 1U);
  EXPECT_EQ(overtake_planner::aw2::nextRaceArmEpoch(41U), 42U);
  EXPECT_EQ(overtake_planner::aw2::nextRaceArmEpoch(
                std::numeric_limits<std::uint64_t>::max()),
            std::nullopt);
}

TEST(Aw2PlanSampleContract, SchemaOneRequiresCompleteAcceptedShadowEvidence) {
  EXPECT_EQ(
      overtake_planner::aw2::identitySchemaVersion(
          true, std::optional<std::uint64_t>{7U}, BindingObservation::ACCEPTED),
      1U);
  EXPECT_EQ(overtake_planner::aw2::identitySchemaVersion(
                true, std::optional<std::uint64_t>{7U},
                BindingObservation::CONSISTENT),
            1U);
  EXPECT_EQ(overtake_planner::aw2::identitySchemaVersion(
                false, std::optional<std::uint64_t>{7U},
                BindingObservation::ACCEPTED),
            0U);
  EXPECT_EQ(overtake_planner::aw2::identitySchemaVersion(
                true, std::nullopt, BindingObservation::ACCEPTED),
            0U);
  EXPECT_EQ(overtake_planner::aw2::identitySchemaVersion(
                true, std::optional<std::uint64_t>{7U},
                BindingObservation::SAME_GENERATION_PAYLOAD_MUTATION),
            0U);
}

TEST(Aw2PlanSampleContract,
     V2PassingEvidenceChainProducesSchemaOneAndAcceptedDelivery) {
  constexpr bool kAuthorityEligible = false;
  const auto request_schema_version =
      [](std::uint8_t identity_schema_version, bool canonical_record_valid,
         DeliveryObservation delivery_observation) {
        const bool delivery_accepted =
            delivery_observation == DeliveryObservation::ACCEPTED ||
            delivery_observation == DeliveryObservation::CONSISTENT_DUPLICATE;
        return identity_schema_version == 1U && canonical_record_valid &&
                       delivery_accepted
                   ? 1U
                   : 0U;
      };

  auto source = validV2CanonicalSource();
  // PASSING here means a stable active transaction and does not grant motion
  // authority. Keep the fixture rejected and STOP-constrained on purpose.
  source.safety_evaluated = true;
  source.feasible = false;
  source.reject_reason = "fixture_shadow_not_authorized";
  const auto source_wire =
      overtake_planner::aw2::canonicalizeV2SourceWire(source);
  ASSERT_TRUE(source_wire.has_value());

  ConnectorTransactionSequencer sequencer;
  ASSERT_TRUE(sequencer.resetForRaceEpoch(3U));
  const auto connector_transaction_id =
      sequencer.update(3U, 11U, "D2", -1, true);
  ASSERT_EQ(connector_transaction_id, std::optional<std::uint64_t>{1U});
  EXPECT_EQ(sequencer.update(3U, 11U, "D2", -1, true),
            connector_transaction_id);

  auto content = validContent();
  content.key.transaction.connector_transaction_id =
      connector_transaction_id.value();
  content.source_kind = CandidateSourceKind::V2_TRAJECTORY;
  content.geometry_point_count = source.x.size();
  content.source_wire = source_wire.value();
  const auto canonical_content =
      overtake_planner::aw2::canonicalizeCandidateContentV1(content);
  ASSERT_TRUE(canonical_content.valid());

  CandidateBinding binding;
  binding.race_arm_epoch = content.key.transaction.race_arm_epoch;
  binding.planner_instance_id = content.key.transaction.planner_instance_id;
  binding.plan_generation = content.key.plan_generation;
  binding.candidate_revision = content.candidate_revision;
  binding.candidate_content_sha256 = canonical_content.sha256;

  SameGenerationBindingTracker binding_tracker;
  const auto binding_observation = binding_tracker.observe(binding);
  ASSERT_EQ(binding_observation, BindingObservation::ACCEPTED);
  const auto identity_schema_version =
      overtake_planner::aw2::identitySchemaVersion(
          true, connector_transaction_id, binding_observation);
  ASSERT_EQ(identity_schema_version, 1U);

  auto record = validExecutionRecord();
  record.key = content.key;
  record.candidate_revision = content.candidate_revision;
  record.candidate_content_sha256 = canonical_content.sha256;
  record.authorization_state =
      overtake_planner::aw2::AuthorizationState::NOT_AUTHORIZED;
  record.trajectory_authorized_legacy = false;
  record.geometry_points.clear();
  for (std::size_t index = 0U; index < source.x.size(); ++index) {
    overtake_planner::aw2::CandidateExecutionPoint point;
    point.time_sec = static_cast<std::int32_t>(source.t[index]);
    point.time_nanosec = static_cast<std::uint32_t>(
        (source.t[index] - static_cast<double>(point.time_sec)) * 1.0e9);
    point.position_x_m = source.x[index];
    point.position_y_m = source.y[index];
    point.orientation_z = std::sin(0.5 * source.yaw[index]);
    point.orientation_w = std::cos(0.5 * source.yaw[index]);
    point.longitudinal_velocity_mps = static_cast<float>(source.v_ref[index]);
    record.geometry_points.push_back(point);
  }
  record.source_kind = CandidateSourceKind::V2_TRAJECTORY;
  record.source_generation = content.key.plan_generation;
  record.source_original_size_bytes =
      static_cast<std::uint32_t>(source_wire->size());
  record.canonical_source_wire = source_wire.value();
  record.constraint_plan_generation = content.key.plan_generation;
  record.constraint_stop_requested = true;
  record.constraint_release_authorized = false;
  record.constraint_speed_limit_mps = 0.0F;
  record.constraint_reason = "fixture_shadow_only";
  record.planned_target_d_m = source.planned_target_d_m;
  record.committed_target_d_m = source.committed_attack_follow_target_d_m;
  record.safety_evaluation_result = SafetyEvaluationResult::REJECTED;
  record.safety_evaluation_reason = source.reject_reason;

  const auto canonical_record =
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(record);
  ASSERT_TRUE(canonical_record.valid());

  DeliveryRecordTracker delivery_tracker;
  const auto delivery_observation = delivery_tracker.observe(
      record.key, canonical_record.plan_sample_record_sha256);
  EXPECT_EQ(delivery_observation, DeliveryObservation::ACCEPTED);
  EXPECT_EQ(request_schema_version(identity_schema_version,
                                   canonical_record.valid(),
                                   delivery_observation),
            1U);
  EXPECT_FALSE(kAuthorityEligible);
  EXPECT_EQ(record.authorization_state,
            overtake_planner::aw2::AuthorizationState::NOT_AUTHORIZED);
  EXPECT_FALSE(record.trajectory_authorized_legacy);
  EXPECT_TRUE(record.constraint_stop_requested);
  EXPECT_FALSE(record.constraint_release_authorized);

  const auto consistent_binding = binding_tracker.observe(binding);
  EXPECT_EQ(consistent_binding, BindingObservation::CONSISTENT);
  const auto duplicate_delivery = delivery_tracker.observe(
      record.key, canonical_record.plan_sample_record_sha256);
  EXPECT_EQ(duplicate_delivery, DeliveryObservation::CONSISTENT_DUPLICATE);
  EXPECT_EQ(request_schema_version(
                overtake_planner::aw2::identitySchemaVersion(
                    true, connector_transaction_id, consistent_binding),
                canonical_record.valid(), duplicate_delivery),
            1U);

  EXPECT_EQ(sequencer.update(3U, 0U, "D2", -1, true), std::nullopt);
  EXPECT_EQ(sequencer.update(3U, 12U, "D2", 0, true), std::nullopt);
  EXPECT_EQ(overtake_planner::aw2::identitySchemaVersion(
                true, std::nullopt, BindingObservation::ACCEPTED),
            0U);

  auto mutated_content = content;
  mutated_content.source_wire.back() ^= 0x01U;
  const auto canonical_mutation =
      overtake_planner::aw2::canonicalizeCandidateContentV1(mutated_content);
  ASSERT_TRUE(canonical_mutation.valid());
  auto mutated_binding = binding;
  mutated_binding.candidate_content_sha256 = canonical_mutation.sha256;
  const auto mutation_observation = binding_tracker.observe(mutated_binding);
  EXPECT_EQ(mutation_observation,
            BindingObservation::SAME_GENERATION_PAYLOAD_MUTATION);
  EXPECT_EQ(overtake_planner::aw2::identitySchemaVersion(
                true, connector_transaction_id, mutation_observation),
            0U);

  auto nonfinite_source = source;
  nonfinite_source.score = std::numeric_limits<double>::infinity();
  EXPECT_EQ(overtake_planner::aw2::canonicalizeV2SourceWire(nonfinite_source),
            std::nullopt);

  auto invalid_record = record;
  invalid_record.planned_target_d_m = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_canonical_record =
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(
          invalid_record);
  EXPECT_FALSE(invalid_canonical_record.valid());
  EXPECT_EQ(request_schema_version(identity_schema_version,
                                   invalid_canonical_record.valid(),
                                   DeliveryObservation::ACCEPTED),
            0U);

  auto conflicting_record = record;
  conflicting_record.constraint_reason = "fixture_same_key_conflict";
  const auto canonical_conflict =
      overtake_planner::aw2::canonicalizeCandidateExecutionRecordV1(
          conflicting_record);
  ASSERT_TRUE(canonical_conflict.valid());
  const auto conflict_observation = delivery_tracker.observe(
      conflicting_record.key, canonical_conflict.plan_sample_record_sha256);
  EXPECT_EQ(conflict_observation, DeliveryObservation::CONFLICT);
  EXPECT_EQ(request_schema_version(identity_schema_version,
                                   canonical_conflict.valid(),
                                   conflict_observation),
            0U);
}

TEST(Aw2PlanSampleContract,
     CurrentDStopKeepsLatchedTransactionSideWithoutChangingPublishedSide) {
  EXPECT_EQ(overtake_planner::aw2::transactionPassDirection(0, true, -1), -1);
  EXPECT_EQ(overtake_planner::aw2::transactionPassDirection(0, true, 1), 1);
  EXPECT_EQ(overtake_planner::aw2::transactionPassDirection(0, false, -1), 0);
  EXPECT_EQ(overtake_planner::aw2::transactionPassDirection(-1, true, 1), -1);
}

} // namespace
