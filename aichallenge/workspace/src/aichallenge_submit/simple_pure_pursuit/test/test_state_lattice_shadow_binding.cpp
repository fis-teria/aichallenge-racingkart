#include "simple_pure_pursuit/state_lattice_shadow_binding.hpp"

#include <gtest/gtest.h>

namespace contract = overtake_transport_contract::c002ay0;
namespace pp = simple_pure_pursuit;

namespace {

using Authorized = multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory;
using Base = multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot;

builtin_interfaces::msg::Time time(std::int32_t sec,
                                   std::uint32_t nanosec = 0U) {
  builtin_interfaces::msg::Time value;
  value.sec = sec;
  value.nanosec = nanosec;
  return value;
}

contract::Digest digest(std::uint8_t value) {
  contract::Digest result{};
  result.fill(value);
  return result;
}

void recanonicalize(Base *snapshot) {
  ASSERT_NE(snapshot, nullptr);
  const auto source = contract::canonicalizeBaseSourceV1(
      snapshot->base_source_kind, snapshot->frame_id,
      snapshot->base_source_stamp, snapshot->base_source_generation,
      snapshot->base_original_point_count, snapshot->base_points);
  ASSERT_TRUE(source.valid());
  snapshot->base_source_sha256 = source.sha256;
  const auto canonical = contract::canonicalizeBaseSnapshotV1(*snapshot);
  ASSERT_TRUE(canonical.valid());
  snapshot->base_geometry_sha256 = canonical.geometry_sha256;
  snapshot->snapshot_sha256 = canonical.sha256;
}

Base base() {
  Base value;
  value.schema_version = Base::SCHEMA_V1_SHADOW;
  value.authority_eligible = false;
  value.record_stamp = time(10, 10000000U);
  value.frame_id = "map";
  value.race_arm_epoch = 3U;
  value.controller_instance_id = 5U;
  value.controller_sequence = 7U;
  value.base_lease_id = 11U;
  value.lease_valid_until = time(11);
  value.base_source_kind = Base::SOURCE_REFERENCE_TRAJECTORY;
  value.base_source_stamp = time(10);
  value.base_source_generation = 13U;
  value.base_original_point_count = 2U;
  value.first_source_index = 0U;
  value.last_source_index = 1U;
  value.nearest_source_index = 0U;
  for (std::uint32_t index = 0U; index < 2U; ++index) {
    multi_purpose_mpc_ros_msgs::msg::CandidateExecutionPoint point;
    point.time_from_start.nanosec = index * 100000000U;
    point.position_x_m = static_cast<double>(index) * 0.25;
    point.orientation_w = 1.0;
    point.longitudinal_velocity_mps = 1.0F;
    value.base_points.push_back(point);
  }
  value.base_source_digest_state = Base::BASE_SOURCE_DIGEST_COMPLETE;
  value.canonical_algorithm_version = 1U;
  value.controller_implementation_sha256 = digest(0x41U);
  value.controller_config_sha256 = digest(0x51U);
  recanonicalize(&value);
  return value;
}

void recanonicalize(Authorized *trajectory) {
  ASSERT_NE(trajectory, nullptr);
  const auto geometry = contract::canonicalizeGeometryV1(
      trajectory->points, "C002AY0_AUTHORIZED_GEOMETRY_V1");
  ASSERT_TRUE(geometry.valid());
  trajectory->geometry_sha256 = geometry.sha256;
  const auto canonical = contract::canonicalizeAuthorizedTrajectoryV1(*trajectory);
  ASSERT_TRUE(canonical.valid());
  trajectory->candidate_start_control_pose_sha256 = canonical.control_pose_sha256;
  trajectory->safety_proof_sha256 = canonical.safety_proof_sha256;
  const auto complete = contract::canonicalizeAuthorizedTrajectoryV1(*trajectory);
  ASSERT_TRUE(complete.valid());
  trajectory->payload_sha256 = complete.sha256;
}

Authorized proposalFrom(const Base &snapshot) {
  Authorized value;
  value.schema_version = Authorized::SCHEMA_V1_SHADOW;
  value.authority_eligible = false;
  value.plan_stamp = time(10, 20000000U);
  value.frame_id = "map";
  value.plan_sample_key.race_arm_epoch = snapshot.race_arm_epoch;
  value.plan_sample_key.planner_instance_id = 17U;
  value.plan_sample_key.attempt_id = 19U;
  value.plan_sample_key.target_vehicle_id = "d2";
  value.plan_sample_key.pass_direction = 1;
  value.plan_sample_key.connector_transaction_id = 23U;
  value.plan_sample_key.plan_stamp = value.plan_stamp;
  value.plan_sample_key.plan_generation = 29U;
  value.candidate_revision = 31U;
  value.authority_token = 37U;
  value.candidate_type = Authorized::CANDIDATE_PASS_LEFT;
  value.phase = Authorized::PHASE_PASSING;
  value.authorization_state = Authorized::AUTHORIZATION_AUTHORIZED;
  value.source_controller_instance_id = snapshot.controller_instance_id;
  value.source_controller_sequence = snapshot.controller_sequence;
  value.base_lease_id = snapshot.base_lease_id;
  value.base_lease_valid_until = snapshot.lease_valid_until;
  value.base_source_kind = snapshot.base_source_kind;
  value.base_source_stamp = snapshot.base_source_stamp;
  value.base_source_generation = snapshot.base_source_generation;
  value.base_original_point_count = snapshot.base_original_point_count;
  value.base_first_source_index = snapshot.first_source_index;
  value.base_last_source_index = snapshot.last_source_index;
  value.base_nearest_source_index = snapshot.nearest_source_index;
  value.base_source_digest_state = snapshot.base_source_digest_state;
  value.canonical_algorithm_version = snapshot.canonical_algorithm_version;
  value.base_geometry_sha256 = snapshot.base_geometry_sha256;
  value.base_source_sha256 = snapshot.base_source_sha256;
  value.base_snapshot_sha256 = snapshot.snapshot_sha256;
  value.points = snapshot.base_points;
  value.original_candidate_point_count = 2U;
  value.total_arc_length_m = 0.25;
  value.required_spatial_horizon_m = 0.25;
  value.join_end_arc_length_m = 0.25;
  value.post_join_arc_length_m = 0.0;
  value.safety_snapshot_id = 41U;
  value.safety_evaluation_result = Authorized::SAFETY_PASSED;
  value.safety_evaluation_stamp = time(10, 15000000U);
  value.safety_valid_until = time(10, 900000000U);
  value.world_safety_snapshot_sha256 = digest(0x61U);
  value.safety_evaluator_implementation_sha256 = digest(0x71U);
  value.safety_evaluator_config_sha256 = digest(0x81U);
  value.controller_implementation_sha256 = snapshot.controller_implementation_sha256;
  value.controller_config_sha256 = snapshot.controller_config_sha256;
  value.candidate_start_control_pose.orientation.w = 1.0;
  value.candidate_start_control_pose_stamp = value.plan_stamp;
  recanonicalize(&value);
  return value;
}

}  // namespace

TEST(StateLatticeShadowRaceArmEpoch, OnlyRisingEdgesAdvanceObserverEpoch) {
  pp::StateLatticeShadowRaceArmEpoch epoch;
  EXPECT_FALSE(epoch.armed());
  EXPECT_EQ(epoch.epoch(), 0U);
  epoch.observe(false);
  EXPECT_FALSE(epoch.armed());
  EXPECT_EQ(epoch.epoch(), 0U);
  epoch.observe(true);
  EXPECT_TRUE(epoch.armed());
  EXPECT_EQ(epoch.epoch(), 1U);
  epoch.observe(true);
  EXPECT_EQ(epoch.epoch(), 1U);
  epoch.observe(false);
  EXPECT_FALSE(epoch.armed());
  EXPECT_EQ(epoch.epoch(), 1U);
  epoch.observe(true);
  EXPECT_TRUE(epoch.armed());
  EXPECT_EQ(epoch.epoch(), 2U);
}

TEST(StateLatticeShadowBinding, ExactCurrentIgnoresPerCycleWitnessFields) {
  const auto original = base();
  const auto proposal = proposalFrom(original);
  auto current = original;
  current.record_stamp = time(10, 30000000U);
  current.controller_sequence += 1U;
  current.base_lease_id += 1U;
  current.lease_valid_until = time(11, 100000000U);
  recanonicalize(&current);

  const auto result = pp::evaluateStateLatticeShadowBinding(
      current, std::nullopt, proposal, time(10, 40000000U));
  EXPECT_EQ(result.disposition,
            pp::StateLatticeShadowBindingDisposition::kExactCurrent);
  EXPECT_FALSE(result.lateral_authority_eligible);
}

TEST(StateLatticeShadowBinding, NormalNMinusOneIsDeferredWithoutAuthority) {
  const auto previous = base();
  const auto proposal = proposalFrom(previous);
  auto current = previous;
  current.nearest_source_index = 1U;
  current.record_stamp = time(10, 30000000U);
  current.controller_sequence += 1U;
  recanonicalize(&current);

  const auto result = pp::evaluateStateLatticeShadowBinding(
      current, previous, proposal, time(10, 40000000U));
  EXPECT_EQ(result.disposition,
            pp::StateLatticeShadowBindingDisposition::kDeferredPredecessor);
  EXPECT_FALSE(result.lateral_authority_eligible);
}

TEST(StateLatticeShadowBinding, NearestIndexAdvanceRequiresFreshProposal) {
  const auto proposal_base = base();
  const auto proposal = proposalFrom(proposal_base);
  auto current = proposal_base;
  current.nearest_source_index = 1U;
  recanonicalize(&current);

  const auto result = pp::evaluateStateLatticeShadowBinding(
      current, std::nullopt, proposal, time(10, 40000000U));
  EXPECT_EQ(
      result.disposition,
      pp::StateLatticeShadowBindingDisposition::kReplanRequiredNearestAdvanced);
}

TEST(StateLatticeShadowBinding, IndexRegressionIsNotTreatedAsLapWrap) {
  const auto current = base();
  auto proposal_base = current;
  proposal_base.nearest_source_index = 1U;
  recanonicalize(&proposal_base);
  const auto proposal = proposalFrom(proposal_base);

  const auto result = pp::evaluateStateLatticeShadowBinding(
      current, std::nullopt, proposal, time(10, 40000000U));
  EXPECT_EQ(
      result.disposition,
      pp::StateLatticeShadowBindingDisposition::kReplanRequiredNearestRegressed);
}

TEST(StateLatticeShadowBinding, SameStampGenerationMutationIsRejected) {
  const auto current = base();
  auto proposal = proposalFrom(current);
  proposal.base_source_sha256[0] ^= 0x01U;
  recanonicalize(&proposal);

  const auto result = pp::evaluateStateLatticeShadowBinding(
      current, std::nullopt, proposal, time(10, 40000000U));
  EXPECT_EQ(result.disposition,
            pp::StateLatticeShadowBindingDisposition::kRejectedSourceMutation);
}

TEST(StateLatticeShadowBinding, NewSourceDeliveryIsDeferredAndStaleIsRejected) {
  const auto current = base();
  auto proposal = proposalFrom(current);
  ++proposal.base_source_generation;
  proposal.base_source_sha256 = digest(0x91U);
  recanonicalize(&proposal);
  auto result = pp::evaluateStateLatticeShadowBinding(
      current, std::nullopt, proposal, time(10, 40000000U));
  EXPECT_EQ(result.disposition,
            pp::StateLatticeShadowBindingDisposition::kDeferredUnpairedDelivery);

  proposal = proposalFrom(current);
  proposal.safety_valid_until = time(10, 40000000U);
  recanonicalize(&proposal);
  result = pp::evaluateStateLatticeShadowBinding(
      current, std::nullopt, proposal, time(10, 40000000U));
  EXPECT_EQ(result.disposition,
            pp::StateLatticeShadowBindingDisposition::kRejectedStale);
}

TEST(StateLatticeShadowBinding, ExpiredProposalBaseLeaseIsRejected) {
  const auto current = base();
  auto proposal = proposalFrom(current);
  proposal.base_lease_valid_until = time(10, 40000000U);
  recanonicalize(&proposal);

  const auto result = pp::evaluateStateLatticeShadowBinding(
      current, std::nullopt, proposal, time(10, 40000000U));
  EXPECT_EQ(result.disposition,
            pp::StateLatticeShadowBindingDisposition::kRejectedStale);
  EXPECT_FALSE(result.lateral_authority_eligible);
}

TEST(StateLatticeShadowBinding, ExpiredPredecessorLeaseIsRejected) {
  auto previous = base();
  previous.lease_valid_until = time(10, 40000000U);
  recanonicalize(&previous);
  auto proposal = proposalFrom(previous);
  proposal.base_lease_valid_until = time(11);
  recanonicalize(&proposal);
  auto current = previous;
  current.nearest_source_index = 1U;
  current.record_stamp = time(10, 30000000U);
  current.controller_sequence += 1U;
  current.lease_valid_until = time(11);
  recanonicalize(&current);

  const auto result = pp::evaluateStateLatticeShadowBinding(
      current, previous, proposal, time(10, 40000000U));
  EXPECT_EQ(result.disposition,
            pp::StateLatticeShadowBindingDisposition::kRejectedStale);
  EXPECT_FALSE(result.lateral_authority_eligible);
}
