#include "state_lattice_overtake_planner/c002ay0_shadow_proposal.hpp"
#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_capture.hpp"
#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_worker.hpp"
#include "state_lattice_overtake_planner/lattice_planner.hpp"
#include "state_lattice_overtake_planner/state_lattice_authority_projection.hpp"

#include "fixtures/dev3_20260728_010706.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

namespace sl = state_lattice_overtake_planner;
namespace contract = overtake_transport_contract::c002ay0;
using Base = multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot;

namespace {

builtin_interfaces::msg::Time time(std::int32_t sec,
                                   std::uint32_t nanosec = 0U) {
  builtin_interfaces::msg::Time result;
  result.sec = sec;
  result.nanosec = nanosec;
  return result;
}

contract::Digest filledDigest(std::uint8_t value) {
  contract::Digest digest;
  digest.fill(value);
  return digest;
}

Base makeBaseSnapshot() {
  Base snapshot;
  snapshot.schema_version = Base::SCHEMA_V1_SHADOW;
  snapshot.authority_eligible = false;
  snapshot.record_stamp = time(100, 10000000U);
  snapshot.frame_id = "map";
  snapshot.race_arm_epoch = 7U;
  snapshot.controller_instance_id = 11U;
  snapshot.controller_sequence = 13U;
  snapshot.base_lease_id = 17U;
  snapshot.lease_valid_until = time(101);
  snapshot.base_source_kind = Base::SOURCE_REFERENCE_TRAJECTORY;
  snapshot.base_source_stamp = time(100);
  snapshot.base_source_generation = 19U;
  snapshot.base_original_point_count = 64U;
  snapshot.first_source_index = 0U;
  snapshot.last_source_index = 63U;
  snapshot.nearest_source_index = 30U;
  snapshot.base_points.reserve(64U);
  for (std::size_t index = 0U; index < 64U; ++index) {
    multi_purpose_mpc_ros_msgs::msg::CandidateExecutionPoint point;
    point.time_from_start.nanosec =
        static_cast<std::uint32_t>(index * 10000000U);
    point.position_x_m = static_cast<double>(index) * 0.1;
    point.position_y_m = 0.0;
    point.orientation_w = 1.0;
    point.longitudinal_velocity_mps = 1.0F;
    snapshot.base_points.push_back(point);
  }
  snapshot.base_source_digest_state = Base::BASE_SOURCE_DIGEST_COMPLETE;
  snapshot.canonical_algorithm_version = 1U;
  const auto source = contract::canonicalizeBaseSourceV1(
      snapshot.base_source_kind, snapshot.frame_id, snapshot.base_source_stamp,
      snapshot.base_source_generation, snapshot.base_original_point_count,
      snapshot.base_points);
  EXPECT_TRUE(source.valid());
  snapshot.base_source_sha256 = source.sha256;
  snapshot.controller_implementation_sha256 = filledDigest(0x41U);
  snapshot.controller_config_sha256 = filledDigest(0x51U);
  const auto canonical = contract::canonicalizeBaseSnapshotV1(snapshot);
  EXPECT_TRUE(canonical.valid());
  snapshot.base_geometry_sha256 = canonical.geometry_sha256;
  snapshot.snapshot_sha256 = canonical.sha256;
  EXPECT_EQ(contract::validateBaseSnapshotV1(snapshot),
            contract::ValidationError::NONE);
  return snapshot;
}

struct D2Context {
  sl::PlannerConfig config;
  sl::FrenetFrame frame;
  sl::GridMap map;
  sl::EgoState ego;
  std::vector<sl::OpponentState> opponents;
  sl::CandidateTrajectory candidate;
};

D2Context makeD2Context() {
  D2Context context;
  const std::string share = TEST_MPC_SOURCE_DIR;
  std::string error;
  EXPECT_TRUE(context.frame.loadCsv(
      share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  EXPECT_TRUE(
      context.map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       context.config, &error))
      << error;
  EXPECT_TRUE(context.map.buildReferenceLayer(context.frame, context.config));

  const auto &fixture = sl::test_fixture::kD2FirstOutputContract;
  context.ego.x = fixture.ego.x_m;
  context.ego.y = fixture.ego.y_m;
  context.ego.yaw = fixture.ego.yaw_rad;
  context.ego.speed_mps = fixture.ego.speed_mps;
  context.ego.stamp_sec = 100.0;
  context.ego.frenet =
      context.frame.project(context.ego.x, context.ego.y, context.ego.yaw);
  context.ego.valid = context.ego.frenet.valid;
  EXPECT_TRUE(context.ego.valid);
  for (const auto &source : fixture.opponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.frenet = context.frame.project(opponent.x, opponent.y, 0.0);
    EXPECT_TRUE(opponent.frenet.valid);
    opponent.yaw = context.frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet =
        context.frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.speed_mps = source.speed_mps;
    opponent.stamp_sec = 100.0;
    opponent.uncertainty_x_m = fixture.opponent_uncertainty_x_m;
    opponent.uncertainty_y_m = fixture.opponent_uncertainty_y_m;
    opponent.valid = opponent.frenet.valid;
    context.opponents.push_back(opponent);
  }
  sl::LatticePlanner planner(context.config, &context.frame, &context.map);
  (void)planner.update(context.ego, context.opponents, true, 100.0);
  const auto candidate =
      std::find_if(planner.candidates().begin(), planner.candidates().end(),
                   [](const sl::CandidateTrajectory &value) {
                     return value.feasible && !value.dense.empty();
                   });
  EXPECT_NE(candidate, planner.candidates().end());
  if (candidate != planner.candidates().end()) {
    context.candidate = *candidate;
  }
  return context;
}

sl::Ay0ShadowProposalIdentity identity() {
  sl::Ay0ShadowProposalIdentity value;
  value.planner_instance_id = 23U;
  value.attempt_id = 29U;
  value.connector_transaction_id = 31U;
  value.authority_token = 37U;
  value.safety_snapshot_id = 41U;
  value.plan_generation = 43U;
  value.candidate_revision = 47U;
  value.target_id = "d1";
  return value;
}

sl::Ay0ShadowSafetyEvidence evidence(const D2Context &context) {
  sl::Ay0ShadowSafetyEvidence value;
  value.evaluator_implementation_sha256 =
      sl::stateLatticeSafetyEvaluatorImplementationDigest();
  value.evaluator_config_sha256 = sl::stateLatticeSafetyEvaluatorConfigDigest(
      context.config, context.map, context.frame);
  return value;
}

sl::c002ay0_shadow::FixedProposalWorkerConfig
workerConfig(const D2Context &context) {
  sl::c002ay0_shadow::FixedProposalWorkerConfig config;
  config.enabled = true;
  config.executable_path = TEST_C002AY0_SHADOW_WORKER_PATH;
  config.session_generation = 53U;
  config.session_nonce = 59U;
  config.static_config.safety_evaluation_enabled = 1U;
  config.static_config.frame_size = 3U;
  config.static_config.frame[0] = 'm';
  config.static_config.frame[1] = 'a';
  config.static_config.frame[2] = 'p';
  config.static_config.wheel_base_m = context.config.wheel_base_m;
  config.static_config.ego_stale_sec = context.config.ego_stale_sec;
  config.static_config.evaluator_implementation_sha256 =
      sl::stateLatticeSafetyEvaluatorImplementationDigest();
  config.static_config.evaluator_config_sha256 =
      sl::stateLatticeSafetyEvaluatorConfigDigest(context.config, context.map,
                                                  context.frame);
  return config;
}

} // namespace

TEST(C002Ay0ShadowProposal, D2DenseCandidatePreservesExactCartesianAndBase) {
  const auto context = makeD2Context();
  const auto base = makeBaseSnapshot();
  const auto result = sl::buildAy0ShadowProposal(
      base, context.candidate, context.ego, context.opponents, context.config,
      evidence(context), time(100, 30000000U), identity());

  ASSERT_TRUE(result.valid()) << result.reason;
  const auto &trajectory = result.trajectory;
  EXPECT_FALSE(trajectory.authority_eligible);
  EXPECT_EQ(trajectory.base_first_source_index, base.first_source_index);
  EXPECT_EQ(trajectory.base_last_source_index, base.last_source_index);
  EXPECT_EQ(trajectory.base_nearest_source_index, base.nearest_source_index);
  EXPECT_EQ(trajectory.base_geometry_sha256, base.base_geometry_sha256);
  EXPECT_EQ(trajectory.base_source_sha256, base.base_source_sha256);
  EXPECT_EQ(trajectory.base_snapshot_sha256, base.snapshot_sha256);
  EXPECT_EQ(trajectory.safety_valid_until.sec, 100);
  EXPECT_EQ(trajectory.safety_valid_until.nanosec, 130000000U);
  ASSERT_EQ(trajectory.points.size(), context.candidate.dense.size());
  for (std::size_t index = 0U; index < trajectory.points.size(); ++index) {
    const auto &actual = trajectory.points[index];
    const auto &expected = context.candidate.dense[index];
    EXPECT_DOUBLE_EQ(actual.position_x_m, expected.x);
    EXPECT_DOUBLE_EQ(actual.position_y_m, expected.y);
    EXPECT_FLOAT_EQ(actual.longitudinal_velocity_mps,
                    static_cast<float>(expected.speed_mps));
    const double actual_yaw =
        2.0 * std::atan2(actual.orientation_z, actual.orientation_w);
    EXPECT_NEAR(actual_yaw, expected.yaw, 1.0e-12);
  }
  EXPECT_DOUBLE_EQ(trajectory.candidate_start_control_pose.position.x,
                   context.ego.x);
  EXPECT_DOUBLE_EQ(trajectory.candidate_start_control_pose.position.y,
                   context.ego.y);
  EXPECT_EQ(contract::validateTrajectoryAgainstBaseSnapshotV1(base, trajectory),
            contract::ValidationError::NONE);
}

TEST(StateLatticeAuthorityProjection,
     SafetyEvaluatedCandidateReleasesWithoutControllerFeedbackRoundTrip) {
  const auto context = makeD2Context();
  const auto built = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), context.candidate, context.ego, context.opponents,
      context.config, evidence(context), time(100, 30000000U), identity());
  ASSERT_TRUE(built.valid()) << built.reason;

  const auto initial = sl::projectStateLatticeAuthority(
      built.trajectory, nullptr, nullptr, false, true,
      built.trajectory.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(initial.valid) << initial.reason;
  EXPECT_FALSE(initial.warmup);
  EXPECT_EQ(initial.plan.lateral_stop_authority_kind,
            initial.plan.LATERAL_STOP_NONE);
  EXPECT_FALSE(initial.constraint.stop_requested);
  EXPECT_TRUE(initial.constraint.release_authorized);
  EXPECT_GT(initial.constraint.speed_limit_mps, 0.0F);
  EXPECT_EQ(initial.plan.candidate_content_sha256,
            built.trajectory.geometry_sha256);
  ASSERT_EQ(initial.plan.trajectory.points.size(),
            built.trajectory.points.size());

  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus tracking;
  tracking.plan_generation = built.trajectory.plan_sample_key.plan_generation;
  tracking.pp_command_fresh = true;
  tracking.pp_command_binding_valid = true;
  tracking.trajectory_tracking_usable = true;
  tracking.pass_warmup_motion_ready = true;
  tracking.pass_warmup_steering_acquisition_active = false;
  tracking.lateral_stop_authority_kind =
      initial.plan.LATERAL_STOP_PASS_WARMUP;
  tracking.lateral_stop_transaction_pass_direction =
      built.trajectory.plan_sample_key.pass_direction;
  tracking.lateral_stop_authority_token = built.trajectory.authority_token;
  const auto ready = sl::projectStateLatticeAuthority(
      built.trajectory, &tracking, nullptr, false, true,
      built.trajectory.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(ready.valid) << ready.reason;
  EXPECT_FALSE(ready.warmup);
  EXPECT_EQ(ready.plan.lateral_stop_authority_kind,
            ready.plan.LATERAL_STOP_NONE);
  EXPECT_FALSE(ready.constraint.stop_requested);
  EXPECT_TRUE(ready.constraint.release_authorized);
  EXPECT_FLOAT_EQ(ready.constraint.speed_limit_mps,
                  initial.constraint.speed_limit_mps);
  EXPECT_EQ(ready.plan.candidate_content_sha256,
            initial.plan.candidate_content_sha256);
}

TEST(StateLatticeAuthorityProjection,
     CurrentBoundedCommandReleasesWithoutZeroSpeedSteeringConvergence) {
  const auto context = makeD2Context();
  const auto built = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), context.candidate, context.ego, context.opponents,
      context.config, evidence(context), time(100, 30000000U), identity());
  ASSERT_TRUE(built.valid());

  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus tracking;
  tracking.plan_generation = built.trajectory.plan_sample_key.plan_generation;
  tracking.pp_command_fresh = true;
  tracking.pp_command_binding_valid = true;
  tracking.trajectory_tracking_usable = true;
  tracking.pass_warmup_motion_ready = false;
  tracking.pass_warmup_steering_acquisition_active = true;
  tracking.lateral_stop_authority_kind =
      multi_purpose_mpc_ros_msgs::msg::OvertakePlan::
          LATERAL_STOP_PASS_WARMUP;
  tracking.lateral_stop_transaction_pass_direction =
      built.trajectory.plan_sample_key.pass_direction;
  tracking.lateral_stop_authority_token = built.trajectory.authority_token;

  const auto released = sl::projectStateLatticeAuthority(
      built.trajectory, &tracking, nullptr, false, true,
      built.trajectory.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(released.valid) << released.reason;
  EXPECT_FALSE(released.warmup);
  EXPECT_TRUE(released.constraint.release_authorized);
  EXPECT_FALSE(released.constraint.stop_requested);
  EXPECT_FLOAT_EQ(released.constraint.speed_limit_mps,
                  built.trajectory.points.front().longitudinal_velocity_mps);
}

TEST(StateLatticeAuthorityProjection,
     RaceAuthorityFailsClosedAndTrackingMismatchIsLeftToMux) {
  const auto context = makeD2Context();
  const auto built = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), context.candidate, context.ego, context.opponents,
      context.config, evidence(context), time(100, 30000000U), identity());
  ASSERT_TRUE(built.valid()) << built.reason;
  const auto disarmed = sl::projectStateLatticeAuthority(
      built.trajectory, nullptr, nullptr, false, false,
      built.trajectory.plan_sample_key.race_arm_epoch);
  EXPECT_FALSE(disarmed.valid);

  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus mismatched;
  mismatched.plan_generation =
      built.trajectory.plan_sample_key.plan_generation + 1U;
  mismatched.trajectory_tracking_usable = true;
  mismatched.pass_warmup_motion_ready = true;
  mismatched.lateral_stop_authority_kind =
      multi_purpose_mpc_ros_msgs::msg::OvertakePlan::
          LATERAL_STOP_PASS_WARMUP;
  mismatched.lateral_stop_transaction_pass_direction =
      built.trajectory.plan_sample_key.pass_direction;
  mismatched.lateral_stop_authority_token = built.trajectory.authority_token;
  const auto held = sl::projectStateLatticeAuthority(
      built.trajectory, &mismatched, nullptr, false, true,
      built.trajectory.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(held.valid) << held.reason;
  EXPECT_FALSE(held.warmup);
  EXPECT_FALSE(held.constraint.stop_requested);
  EXPECT_TRUE(held.constraint.release_authorized);
}

TEST(StateLatticeAuthorityProjection,
     FreshDirectPredecessorReadinessReleasesRevalidatedSuccessor) {
  const auto context = makeD2Context();
  const auto built = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), context.candidate, context.ego, context.opponents,
      context.config, evidence(context), time(100, 30000000U), identity());
  ASSERT_TRUE(built.valid()) << built.reason;

  auto successor_identity = identity();
  successor_identity.plan_generation += 2U;
  successor_identity.candidate_revision += 2U;
  ++successor_identity.safety_snapshot_id;
  const auto successor_built = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), context.candidate, context.ego, context.opponents,
      context.config, evidence(context), time(100, 80000000U),
      successor_identity);
  ASSERT_TRUE(successor_built.valid()) << successor_built.reason;
  const auto &successor = successor_built.trajectory;

  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus tracking;
  tracking.plan_generation = built.trajectory.plan_sample_key.plan_generation;
  tracking.pp_command_fresh = true;
  tracking.pp_command_binding_valid = true;
  tracking.trajectory_tracking_usable = true;
  tracking.pass_warmup_motion_ready = true;
  tracking.pass_warmup_steering_acquisition_active = false;
  tracking.lateral_stop_authority_kind =
      multi_purpose_mpc_ros_msgs::msg::OvertakePlan::LATERAL_STOP_PASS_WARMUP;
  tracking.lateral_stop_transaction_pass_direction =
      built.trajectory.plan_sample_key.pass_direction;
  tracking.lateral_stop_authority_token = built.trajectory.authority_token;

  const auto ready = sl::projectStateLatticeAuthority(
      successor, &tracking, &built.trajectory, true, true,
      successor.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(ready.valid) << ready.reason;
  EXPECT_FALSE(ready.warmup);
  EXPECT_EQ(ready.plan.plan_generation,
            successor.plan_sample_key.plan_generation);
  EXPECT_EQ(ready.constraint.plan_generation,
            successor.plan_sample_key.plan_generation);
  EXPECT_EQ(ready.plan.candidate_revision,
            successor.candidate_revision);
  EXPECT_EQ(ready.plan.candidate_content_sha256,
            successor.geometry_sha256);
  EXPECT_EQ(ready.plan.trajectory.points.size(),
            successor.points.size());

  const auto stale = sl::projectStateLatticeAuthority(
      successor, &tracking, &built.trajectory, false, true,
      successor.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(stale.valid) << stale.reason;
  EXPECT_FALSE(stale.warmup);
  EXPECT_TRUE(stale.constraint.release_authorized);
  EXPECT_EQ(stale.plan.plan_generation,
            successor.plan_sample_key.plan_generation);
}

TEST(StateLatticeAuthorityProjection,
     FreshReleasedPredecessorTrackingContinuesReplannedSuccessor) {
  const auto context = makeD2Context();
  const auto built = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), context.candidate, context.ego, context.opponents,
      context.config, evidence(context), time(100, 30000000U), identity());
  ASSERT_TRUE(built.valid()) << built.reason;

  auto successor_identity = identity();
  successor_identity.plan_generation += 2U;
  successor_identity.candidate_revision += 2U;
  ++successor_identity.safety_snapshot_id;
  auto replanned_candidate = context.candidate;
  for (auto &point : replanned_candidate.representative) {
    point.x += 1.0e-5;
  }
  for (auto &point : replanned_candidate.dense) {
    point.x += 1.0e-5;
  }
  auto replanned_ego = context.ego;
  replanned_ego.x += 1.0e-5;
  replanned_ego.frenet =
      context.frame.project(replanned_ego.x, replanned_ego.y,
                            replanned_ego.yaw);
  sl::LatticePlanner evaluator(context.config, &context.frame, &context.map);
  ASSERT_TRUE(evaluator.evaluateTrajectory(
      &replanned_candidate, context.opponents,
      std::abs(replanned_ego.speed_mps)));
  const auto successor_built = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), replanned_candidate, replanned_ego, context.opponents,
      context.config, evidence(context), time(100, 80000000U),
      successor_identity);
  ASSERT_TRUE(successor_built.valid()) << successor_built.reason;
  ASSERT_NE(successor_built.trajectory.geometry_sha256,
            built.trajectory.geometry_sha256);

  multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus tracking;
  tracking.plan_generation = built.trajectory.plan_sample_key.plan_generation;
  tracking.trajectory_tracking_usable = true;
  tracking.pass_warmup_motion_ready = false;
  tracking.pass_warmup_steering_acquisition_active = false;
  tracking.lateral_stop_authority_kind =
      multi_purpose_mpc_ros_msgs::msg::OvertakePlan::LATERAL_STOP_NONE;
  tracking.lateral_stop_transaction_pass_direction = 0;
  tracking.lateral_stop_authority_token = 0U;

  const auto continued = sl::projectStateLatticeAuthority(
      successor_built.trajectory, &tracking, &built.trajectory, true, true,
      successor_built.trajectory.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(continued.valid) << continued.reason;
  EXPECT_FALSE(continued.warmup);
  EXPECT_TRUE(continued.constraint.release_authorized);

  auto changed_transaction_identity = successor_identity;
  ++changed_transaction_identity.connector_transaction_id;
  ++changed_transaction_identity.safety_snapshot_id;
  const auto changed_transaction_built = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), replanned_candidate, replanned_ego, context.opponents,
      context.config, evidence(context), time(100, 90000000U),
      changed_transaction_identity);
  ASSERT_TRUE(changed_transaction_built.valid())
      << changed_transaction_built.reason;
  const auto &changed_transaction = changed_transaction_built.trajectory;
  const auto changed_transaction_result = sl::projectStateLatticeAuthority(
      changed_transaction, &tracking, &built.trajectory, true, true,
      changed_transaction.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(changed_transaction_result.valid)
      << changed_transaction_result.reason;
  EXPECT_FALSE(changed_transaction_result.warmup);
  EXPECT_TRUE(changed_transaction_result.constraint.release_authorized);

  const auto stale = sl::projectStateLatticeAuthority(
      successor_built.trajectory, &tracking, &built.trajectory, false, true,
      successor_built.trajectory.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(stale.valid) << stale.reason;
  EXPECT_FALSE(stale.warmup);
  EXPECT_TRUE(stale.constraint.release_authorized);

  auto later_identity = successor_identity;
  later_identity.plan_generation += 4U;
  later_identity.candidate_revision += 4U;
  later_identity.safety_snapshot_id += 2U;
  const auto later_built = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), context.candidate, context.ego, context.opponents,
      context.config, evidence(context), time(100, 180000000U),
      later_identity);
  ASSERT_TRUE(later_built.valid()) << later_built.reason;
  const auto bounded_continuation = sl::projectStateLatticeAuthority(
      later_built.trajectory, &tracking, &built.trajectory, true, true,
      later_built.trajectory.plan_sample_key.race_arm_epoch);
  ASSERT_TRUE(bounded_continuation.valid) << bounded_continuation.reason;
  EXPECT_FALSE(bounded_continuation.warmup);
}

TEST(StateLatticeAuthorityProjection,
     TransactionIdentityIsStableUntilTargetOrSideChanges) {
  sl::StateLatticeAuthorityTransactionState state;
  const auto *first = state.select("D2", -1);
  ASSERT_NE(first, nullptr);
  const auto first_value = *first;
  const auto *held = state.select("D2", -1);
  ASSERT_NE(held, nullptr);
  EXPECT_EQ(held->attempt_id, first_value.attempt_id);
  EXPECT_EQ(held->connector_transaction_id,
            first_value.connector_transaction_id);
  EXPECT_EQ(held->authority_token, first_value.authority_token);

  const auto *changed = state.select("D2", 1);
  ASSERT_NE(changed, nullptr);
  EXPECT_GT(changed->attempt_id, first_value.attempt_id);
  EXPECT_NE(changed->authority_token, first_value.authority_token);
  EXPECT_EQ(state.select("", 0), nullptr);
}

TEST(C002Ay0ShadowProposal,
     SafetyLeaseCoversTwoPlannerPeriodsButNeverExceedsInputFreshness) {
  auto context = makeD2Context();
  const auto base = makeBaseSnapshot();
  context.config.planner_rate_hz = 20.0;
  context.config.ego_stale_sec = 0.06;
  context.config.opponent_stale_sec = 0.50;
  const auto result = sl::buildAy0ShadowProposal(
      base, context.candidate, context.ego, context.opponents, context.config,
      evidence(context), time(100, 30000000U), identity());

  ASSERT_TRUE(result.valid()) << result.reason;
  EXPECT_EQ(result.trajectory.safety_valid_until.sec, 100);
  EXPECT_EQ(result.trajectory.safety_valid_until.nanosec, 90000000U);
}

TEST(C002Ay0ShadowProposal, TrackabilityProfileIsBoundToConfigProvenance) {
  const auto context = makeD2Context();
  auto pure_pursuit_config = context.config;
  pure_pursuit_config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  auto instant_config = context.config;
  instant_config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::INSTANT;

  const auto shadow_digest = sl::stateLatticeSafetyEvaluatorConfigDigest(
      context.config, context.map, context.frame);
  const auto pure_pursuit_digest = sl::stateLatticeSafetyEvaluatorConfigDigest(
      pure_pursuit_config, context.map, context.frame);
  const auto instant_digest = sl::stateLatticeSafetyEvaluatorConfigDigest(
      instant_config, context.map, context.frame);

  EXPECT_NE(shadow_digest, pure_pursuit_digest);
  EXPECT_NE(shadow_digest, instant_digest);
  EXPECT_NE(pure_pursuit_digest, instant_digest);
}

TEST(C002Ay0ShadowProposal,
     BaseCurrentPreflightMatchesBuilderTimeBoundaries) {
  auto base = makeBaseSnapshot();
  base.record_stamp = time(100, 100000000U);
  base.base_source_stamp = time(100, 50000000U);
  base.lease_valid_until = time(101);

  EXPECT_TRUE(sl::baseSnapshotCurrentForProposal(
      base, time(100, 200000000U), 0.10));
  EXPECT_FALSE(sl::baseSnapshotCurrentForProposal(
      base, time(100, 200000001U), 0.10));

  base.record_stamp = time(100, 900000000U);
  base.base_source_stamp = time(100, 950000000U);
  EXPECT_FALSE(sl::baseSnapshotCurrentForProposal(
      base, time(100, 900000000U), 0.10));

  base.base_source_stamp = time(100, 850000000U);
  EXPECT_FALSE(sl::baseSnapshotCurrentForProposal(base, time(101), 0.10));
  EXPECT_FALSE(sl::baseSnapshotCurrentForProposal(base, time(100), 0.0));
}

TEST(C002Ay0ShadowProposal, InvalidStaleAndMutatedInputsNeverPublishValid) {
  const auto context = makeD2Context();
  const auto safety_evidence = evidence(context);
  const auto proposal_identity = identity();

  auto expired = makeBaseSnapshot();
  expired.lease_valid_until = time(100, 20000000U);
  auto canonical = contract::canonicalizeBaseSnapshotV1(expired);
  expired.base_geometry_sha256 = canonical.geometry_sha256;
  expired.snapshot_sha256 = canonical.sha256;
  const auto expired_result = sl::buildAy0ShadowProposal(
      expired, context.candidate, context.ego, context.opponents,
      context.config, safety_evidence, time(100, 30000000U), proposal_identity);
  EXPECT_EQ(expired_result.failure,
            sl::Ay0ShadowProposalFailure::BASE_NOT_CURRENT);
  EXPECT_FALSE(expired_result.valid());

  auto stale = makeBaseSnapshot();
  stale.record_stamp = time(99, 400000000U);
  stale.base_source_stamp = time(99, 300000000U);
  canonical = contract::canonicalizeBaseSnapshotV1(stale);
  stale.base_geometry_sha256 = canonical.geometry_sha256;
  stale.snapshot_sha256 = canonical.sha256;
  const auto stale_result = sl::buildAy0ShadowProposal(
      stale, context.candidate, context.ego, context.opponents, context.config,
      safety_evidence, time(100, 30000000U), proposal_identity);
  EXPECT_EQ(stale_result.failure,
            sl::Ay0ShadowProposalFailure::BASE_NOT_CURRENT);
  EXPECT_FALSE(stale_result.valid());

  auto future_source = makeBaseSnapshot();
  future_source.record_stamp = time(100, 50000000U);
  future_source.base_source_stamp = time(100, 40000000U);
  canonical = contract::canonicalizeBaseSnapshotV1(future_source);
  future_source.base_geometry_sha256 = canonical.geometry_sha256;
  future_source.snapshot_sha256 = canonical.sha256;
  const auto future_result = sl::buildAy0ShadowProposal(
      future_source, context.candidate, context.ego, context.opponents,
      context.config, safety_evidence, time(100, 30000000U), proposal_identity);
  EXPECT_EQ(future_result.failure,
            sl::Ay0ShadowProposalFailure::BASE_NOT_CURRENT);
  EXPECT_FALSE(future_result.valid());

  auto mutated = makeBaseSnapshot();
  mutated.nearest_source_index += 1U;
  const auto mutated_result = sl::buildAy0ShadowProposal(
      mutated, context.candidate, context.ego, context.opponents,
      context.config, safety_evidence, time(100, 30000000U), proposal_identity);
  EXPECT_EQ(mutated_result.failure, sl::Ay0ShadowProposalFailure::BASE_INVALID);
  EXPECT_FALSE(mutated_result.valid());

  auto infeasible = context.candidate;
  infeasible.feasible = false;
  const auto infeasible_result = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), infeasible, context.ego, context.opponents,
      context.config, safety_evidence, time(100, 30000000U), proposal_identity);
  EXPECT_EQ(infeasible_result.failure,
            sl::Ay0ShadowProposalFailure::CANDIDATE_NOT_FEASIBLE);
  EXPECT_FALSE(infeasible_result.valid());

  auto displaced = context.candidate;
  displaced.dense.front().x += 0.001;
  const auto displaced_result = sl::buildAy0ShadowProposal(
      makeBaseSnapshot(), displaced, context.ego, context.opponents,
      context.config, safety_evidence, time(100, 30000000U), proposal_identity);
  EXPECT_EQ(displaced_result.failure,
            sl::Ay0ShadowProposalFailure::CANDIDATE_START_MISMATCH);
  EXPECT_FALSE(displaced_result.valid());
}

TEST(C002Ay0ShadowProposal, WorkerPublishesOnlyCanonicalShadowPayload) {
  const auto context = makeD2Context();
  const auto base = makeBaseSnapshot();
  const auto worker_config = workerConfig(context);
  overtake_transport_contract::c002ay0::FixedBaseRecord fixed_base;
  sl::c002ay0_shadow::FixedIncomingBaseProvenance base_provenance;
  ASSERT_TRUE(sl::c002ay0_shadow::copyFixedBaseRecord(
      base, worker_config.session_generation, worker_config.session_nonce,
      &fixed_base, &base_provenance));
  multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
      rebuilt_base;
  ASSERT_EQ(
      overtake_transport_contract::c002ay0::buildBaseSnapshotFromFixedRecord(
          fixed_base, rebuilt_base),
      contract::ValidationError::NONE);
  EXPECT_EQ(rebuilt_base.base_source_sha256,
            base_provenance.base_source_sha256);
  EXPECT_EQ(rebuilt_base.base_geometry_sha256,
            base_provenance.base_geometry_sha256);
  EXPECT_EQ(rebuilt_base.snapshot_sha256, base_provenance.base_snapshot_sha256);
  auto digest_mutated = base;
  digest_mutated.nearest_source_index += 1U;
  EXPECT_FALSE(sl::c002ay0_shadow::copyFixedBaseRecord(
      digest_mutated, worker_config.session_generation,
      worker_config.session_nonce, &fixed_base, &base_provenance));
  ASSERT_TRUE(sl::c002ay0_shadow::copyFixedBaseRecord(
      base, worker_config.session_generation, worker_config.session_nonce,
      &fixed_base, &base_provenance));
  sl::c002ay0_shadow::FixedProposalRecord record;
  ASSERT_EQ(sl::c002ay0_shadow::buildFixedProposalRecord(
                fixed_base, base_provenance, context.candidate, context.ego,
                context.opponents, evidence(context), time(100, 30000000U),
                identity(), worker_config.session_generation,
                worker_config.session_nonce, &record),
            sl::c002ay0_shadow::FixedProposalCaptureResult::kBuilt);
  EXPECT_NE(record.capture_monotonic_ns, 0U);

  const bool owns_rclcpp = !rclcpp::ok();
  if (owns_rclcpp) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  auto observer = std::make_shared<rclcpp::Node>(
      "c002ay0_state_lattice_shadow_worker_test_observer");
  std::atomic<bool> received{false};
  multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory observed;
  auto subscription = observer->create_subscription<
      multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory>(
      "/debug/overtake/state_lattice/authorized_cartesian_trajectory",
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      [&received,
       &observed](const multi_purpose_mpc_ros_msgs::msg::
                      AuthorizedCartesianTrajectory::SharedPtr message) {
        observed = *message;
        received.store(true, std::memory_order_release);
      });
  (void)subscription;

  auto session =
      sl::c002ay0_shadow::FixedProposalWorkerSession::start(worker_config);
  ASSERT_NE(session, nullptr);
  ASSERT_EQ(sl::c002ay0_shadow::buildFixedProposalRecord(
                fixed_base, base_provenance, context.candidate, context.ego,
                context.opponents, evidence(context), time(100, 30000000U),
                identity(), worker_config.session_generation,
                worker_config.session_nonce, &record),
            sl::c002ay0_shadow::FixedProposalCaptureResult::kBuilt);
  ASSERT_TRUE(session->tryCapture(record));
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(received.load(std::memory_order_acquire))
      << "worker validation_reject_count=" << session->validationRejectCount();
  if (!received.load(std::memory_order_acquire)) {
    (void)session->shutdown();
    if (owns_rclcpp) {
      rclcpp::shutdown();
    }
    return;
  }
  EXPECT_FALSE(observed.authority_eligible);
  const auto expected = sl::buildAy0ShadowProposal(
      base, context.candidate, context.ego, context.opponents, context.config,
      evidence(context), time(100, 30000000U), identity());
  ASSERT_TRUE(expected.valid()) << expected.reason;
  rclcpp::Serialization<
      multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory>
      serializer;
  rclcpp::SerializedMessage expected_serialized;
  rclcpp::SerializedMessage observed_serialized;
  serializer.serialize_message(&expected.trajectory, &expected_serialized);
  serializer.serialize_message(&observed, &observed_serialized);
  const auto &expected_bytes = expected_serialized.get_rcl_serialized_message();
  const auto &observed_bytes = observed_serialized.get_rcl_serialized_message();
  ASSERT_EQ(observed_bytes.buffer_length, expected_bytes.buffer_length);
  EXPECT_EQ(std::memcmp(observed_bytes.buffer, expected_bytes.buffer,
                        expected_bytes.buffer_length),
            0);
  multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot rebuilt;
  ASSERT_EQ(
      overtake_transport_contract::c002ay0::buildBaseSnapshotFromFixedRecord(
          fixed_base, rebuilt),
      contract::ValidationError::NONE);
  EXPECT_EQ(
      contract::validateTrajectoryAgainstBaseSnapshotV1(rebuilt, observed),
      contract::ValidationError::NONE);
  EXPECT_GE(session->publishedCount(), 1U);
  EXPECT_EQ(session->validationRejectCount(), 0U);
  const auto diagnostics = session->diagnostics();
  EXPECT_TRUE(diagnostics.accepting);
  EXPECT_TRUE(diagnostics.worker_ready);
  EXPECT_GE(diagnostics.published_count, 1U);
  EXPECT_GT(diagnostics.worker_process_age_ns, 0U);
  EXPECT_GT(diagnostics.last_publish_queue_age_ns, 0U);

  // The shadow publisher may publish both captures at its low-latency cadence,
  // or coalesce them when scheduling compresses the interval.  In either case
  // both accepted inputs must be accounted for without a validation reject.
  const auto published_before = session->publishedCount();
  const auto coalesced_before = session->diagnostics().coalesced_count;
  ASSERT_EQ(sl::c002ay0_shadow::buildFixedProposalRecord(
                fixed_base, base_provenance, context.candidate, context.ego,
                context.opponents, evidence(context), time(100, 30000000U),
                identity(), worker_config.session_generation,
                worker_config.session_nonce, &record),
            sl::c002ay0_shadow::FixedProposalCaptureResult::kBuilt);
  ASSERT_TRUE(session->tryCapture(record));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(sl::c002ay0_shadow::buildFixedProposalRecord(
                fixed_base, base_provenance, context.candidate, context.ego,
                context.opponents, evidence(context), time(100, 30000000U),
                identity(), worker_config.session_generation,
                worker_config.session_nonce, &record),
            sl::c002ay0_shadow::FixedProposalCaptureResult::kBuilt);
  ASSERT_TRUE(session->tryCapture(record));
  const auto coalesced_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while ((session->publishedCount() - published_before) +
                 (session->diagnostics().coalesced_count - coalesced_before) <
             2U &&
         std::chrono::steady_clock::now() < coalesced_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GE(session->publishedCount(), published_before + 1U);
  EXPECT_GE((session->publishedCount() - published_before) +
                (session->diagnostics().coalesced_count - coalesced_before),
            2U);

  // Records already older than the worker's 20 ms publication budget are
  // discarded fail-closed instead of being published near their 50 ms safety
  // expiry.  Use a comfortably stale age to avoid a scheduler-sensitive
  // boundary assertion.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_EQ(sl::c002ay0_shadow::buildFixedProposalRecord(
                fixed_base, base_provenance, context.candidate, context.ego,
                context.opponents, evidence(context), time(100, 30000000U),
                identity(), worker_config.session_generation,
                worker_config.session_nonce, &record),
            sl::c002ay0_shadow::FixedProposalCaptureResult::kBuilt);
  const auto monotonic_now_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
  ASSERT_GT(monotonic_now_ns, 50000000U);
  record.capture_monotonic_ns = monotonic_now_ns - 50000000U;
  const auto stale_published_before = session->publishedCount();
  const auto stale_rejected_before = session->validationRejectCount();
  ASSERT_TRUE(session->tryCapture(record));
  const auto stale_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (session->validationRejectCount() == stale_rejected_before &&
         std::chrono::steady_clock::now() < stale_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(session->publishedCount(), stale_published_before);
  EXPECT_EQ(session->validationRejectCount(), stale_rejected_before + 1U);
  (void)session->shutdown();
  if (owns_rclcpp) {
    rclcpp::shutdown();
  }
}
