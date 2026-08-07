#include "overtake_planner/candidate_builder.hpp"
#include "overtake_planner/overtake_supervisor_v2.hpp"
#include "overtake_planner/safety_evaluator.hpp"
#include "overtake_planner/v2_localized_pass_profile_builder.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace {

overtake_planner::CandidateTrajectory
candidate(overtake_planner::CandidateType type, bool feasible,
          double score = 0.0) {
  overtake_planner::CandidateTrajectory value;
  value.type = type;
  value.safety_evaluated = true;
  value.feasible = feasible;
  value.score = score;
  value.x = {0.0, 1.0};
  value.y = {0.0, 0.0};
  value.yaw = {0.0, 0.0};
  value.d = {0.0, 0.0};
  value.v_ref = {2.0, 2.0};
  if (type == overtake_planner::CandidateType::PASS_LEFT) {
    value.planned_target_d_m = 1.0;
  } else if (type == overtake_planner::CandidateType::PASS_RIGHT) {
    value.planned_target_d_m = -1.0;
  }
  return value;
}

overtake_planner::CandidateTrajectory
candidateWithD(overtake_planner::CandidateType type, bool feasible,
               std::vector<double> lateral_offsets) {
  auto value = candidate(type, feasible);
  value.d = std::move(lateral_offsets);
  value.x.resize(value.d.size(), 0.0);
  value.y = value.d;
  value.yaw.resize(value.d.size(), 0.0);
  value.v_ref.resize(value.d.size(), 2.0);
  return value;
}

overtake_planner::LocalizedLateralProfile
passProfile(overtake_planner::CandidateType type,
            const std::string &target_id = "d2", double target_d_m = 1.0) {
  overtake_planner::LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = type;
  profile.target_id = target_id;
  profile.created_time_sec = 1.0;
  profile.anchor_s_m = 0.0;
  profile.ego_unwrapped_s_m = 0.0;
  profile.last_ego_wrapped_s_m = 0.0;
  profile.last_ego_stamp_sec = 1.0;
  profile.target_s_m = 10.0;
  profile.avoid_start_s_m = 4.0;
  profile.full_offset_start_s_m = 7.0;
  profile.avoid_start_before_target_m = 6.0;
  profile.full_offset_before_target_m = 3.0;
  profile.full_offset_end_s_m = 13.0;
  profile.merge_end_s_m = 20.0;
  profile.start_d_m = 0.0;
  profile.target_d_m = target_d_m;
  profile.chain_waypoints = {{target_id, 10.0, target_d_m, 10.0, 10.0, 1.0}};
  profile.chain_tail_id = target_id;
  profile.chain_tail_s_m = 10.0;
  profile.chain_tail_speed_mps = 1.0;
  profile.chain_target_count = 1;
  return profile;
}

overtake_planner::SupervisorV2Input
input(const std::vector<overtake_planner::CandidateTrajectory> &candidates) {
  overtake_planner::SupervisorV2Input value;
  value.target_present = true;
  value.target_vehicle_id = "d2";
  value.safety_inputs_complete = true;
  value.tracking_usable = true;
  value.pass_left_start_allowed = true;
  value.pass_right_start_allowed = true;
  value.abort_release_allowed = true;
  value.candidates = &candidates;
  static const auto left_profile =
      passProfile(overtake_planner::CandidateType::PASS_LEFT, "d2", 1.0);
  static const auto right_profile =
      passProfile(overtake_planner::CandidateType::PASS_RIGHT, "d2", -1.0);
  const auto candidate_for_type =
      [&candidates](overtake_planner::CandidateType type) {
        const auto item = std::find_if(
            candidates.begin(), candidates.end(),
            [type](const overtake_planner::CandidateTrajectory &candidate) {
              return candidate.type == type;
            });
        return item == candidates.end() ? nullptr : &*item;
      };
  value.pass_left_probe = {
      candidate_for_type(overtake_planner::CandidateType::PASS_LEFT),
      &left_profile, "d2"};
  value.pass_right_probe = {
      candidate_for_type(overtake_planner::CandidateType::PASS_RIGHT),
      &right_profile, "d2"};
  return value;
}

overtake_planner::FrenetFrame loopFrame() {
  constexpr double kHalfPi = 1.5707963267948966;
  std::vector<overtake_planner::ReferencePoint> reference = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 5.0},
      {10.0, 10.0, 0.0, 0.0, 0.0, 5.0},
      {20.0, 10.0, 10.0, kHalfPi, 0.0, 5.0},
      {30.0, 0.0, 10.0, -kHalfPi, 0.0, 5.0},
  };
  overtake_planner::FrenetFrame frame;
  frame.setReference(std::move(reference));
  return frame;
}

overtake_planner::EgoState
proposalEgo(const overtake_planner::FrenetFrame &frame) {
  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.stamp_sec = 10.0;
  ego.v = 0.5;
  ego.frenet.s = 2.0;
  ego.frenet.d = 0.0;
  const auto pose = frame.frenetToCartesian(ego.frenet.s, ego.frenet.d);
  ego.x = pose.x;
  ego.y = pose.y;
  ego.yaw = pose.yaw;
  return ego;
}

overtake_planner::PlannerConfig proposalConfig() {
  overtake_planner::PlannerConfig config;
  config.horizon_points = 120;
  config.horizon_dt_sec = 0.05;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.left_offset_m = 1.0;
  config.prepare_distance_m = 20.0;
  config.pass_speed_cap_mps = 2.0;
  config.pass_assumed_accel_mps2 = 1.0;
  return config;
}

TEST(OvertakeSupervisorV2,
     ProposalPassAccelerationIsSafetyEvaluatedWithoutExecutionAuthority) {
  const auto frame = loopFrame();
  const auto config = proposalConfig();
  const auto ego = proposalEgo(frame);
  overtake_planner::BlockedInfo blocked;
  blocked.pass_proposal_acceleration_allowed = true;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_target_index = 0;
  blocked.maneuver_target_id = "d2";
  overtake_planner::OpponentState target;
  target.id = "d2";
  target.valid = true;
  target.stamp_sec = ego.stamp_sec;
  target.v = 0.0;
  target.frenet.s = 30.0;
  target.frenet.d = 0.0;
  const auto target_pose =
      frame.frenetToCartesian(target.frenet.s, target.frenet.d);
  target.x = target_pose.x;
  target.y = target_pose.y;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto proposal = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {target},
      nullptr, false,
      overtake_planner::CandidatePurpose::PROPOSAL_SAFETY_EVALUATION);
  const auto execution =
      builder.makeCandidate(overtake_planner::CandidateType::PASS_LEFT, ego,
                            blocked, {target}, nullptr);

  ASSERT_FALSE(proposal.longitudinal_offsets_m.empty());
  ASSERT_FALSE(execution.longitudinal_offsets_m.empty());
  EXPECT_TRUE(proposal.longitudinal_profile_valid);
  EXPECT_GE(proposal.longitudinal_offsets_m.back(),
            proposal.required_controller_spatial_horizon_m);
  EXPECT_GT(proposal.predicted_speed_mps.back(), ego.v + 1.0e-6);
  EXPECT_NEAR(execution.predicted_speed_mps.back(), ego.v, 1.0e-6);
  EXPECT_GT(proposal.longitudinal_offsets_m.back(),
            execution.longitudinal_offsets_m.back());

  overtake_planner::PredictedOpponent prediction;
  prediction.id = target.id;
  for (const double t_sec : proposal.t) {
    prediction.t.push_back(t_sec);
    prediction.x.push_back(target.x);
    prediction.y.push_back(target.y);
    prediction.s.push_back(target.frenet.s);
    prediction.d.push_back(target.frenet.d);
  }
  overtake_planner::SafetyEvaluator evaluator(frame, config);
  EXPECT_TRUE(evaluator.evaluate(proposal, {prediction}))
      << proposal.reject_reason;
  EXPECT_TRUE(proposal.safety_evaluated);
  EXPECT_TRUE(proposal.feasible);
}

TEST(OvertakeSupervisorV2, ProposalPassAccelerationFailsClosedWhenUnsafe) {
  const auto frame = loopFrame();
  const auto config = proposalConfig();
  const auto ego = proposalEgo(frame);
  overtake_planner::BlockedInfo blocked;
  blocked.pass_proposal_acceleration_allowed = true;
  overtake_planner::CandidateBuilder builder(frame, config);
  auto proposal = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {}, nullptr,
      false, overtake_planner::CandidatePurpose::PROPOSAL_SAFETY_EVALUATION);

  overtake_planner::PredictedOpponent collision;
  collision.id = "d1";
  collision.t = proposal.t;
  collision.x = proposal.x;
  collision.y = proposal.y;
  collision.s = proposal.s;
  collision.d = proposal.d;
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  EXPECT_FALSE(evaluator.evaluate(proposal, {collision}));
  EXPECT_TRUE(proposal.safety_evaluated);
  EXPECT_FALSE(proposal.feasible);
}

TEST(OvertakeSupervisorV2, ExecutionPassPreservesLegacyAccelerationGate) {
  const auto frame = loopFrame();
  const auto config = proposalConfig();
  const auto ego = proposalEgo(frame);
  overtake_planner::BlockedInfo blocked;
  blocked.pass_acceleration_allowed = true;
  overtake_planner::CandidateBuilder builder(frame, config);

  const auto execution = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {}, nullptr);

  ASSERT_FALSE(execution.predicted_speed_mps.empty());
  EXPECT_GT(execution.predicted_speed_mps.back(), ego.v + 1.0e-6);
}

TEST(OvertakeSupervisorV2,
     StartGridPassProfileStartsAtActualPoseAndCanReachOffsetAfterTarget) {
  // 20260726-155212の最初のD1 interaction: ego=(s=28.31,d=2.58)、
  // stationary d3=(ds=4.30,d=1.76)。開始時min_cbf_h=0.196の安全な
  // PASS probeは、static target-s deadlineではなく時系列SafetyEvaluatorまで
  // 評価されなければならない。
  const auto frame = loopFrame();
  overtake_planner::PlannerConfig config;
  config.v_passthrough_mps = 6.0;
  config.opponent_stale_time_sec = 1.0;
  config.input_future_stamp_tolerance_sec = 0.1;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  // actual pose/yaw始端のPASSと相手楕円だけを検証するwide test corridor。
  // Runtime mapのwall marginやfootprint設定は変更しない。
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  overtake_planner::V2LocalizedPassProfileBuilder profile_builder(frame,
                                                                  config);

  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.stamp_sec = 10.0;
  ego.v = 0.0;
  ego.frenet.s = 28.31;
  ego.frenet.d = 2.58;
  const auto ego_pose = frame.frenetToCartesian(ego.frenet.s, ego.frenet.d);
  ego.x = ego_pose.x;
  ego.y = ego_pose.y;
  ego.yaw = ego_pose.yaw;

  overtake_planner::OpponentState d3;
  d3.id = "d3";
  d3.valid = true;
  d3.stamp_sec = 10.0;
  d3.v = 0.0;
  d3.frenet.s = ego.frenet.s + 4.30;
  d3.frenet.d = 1.76;
  const auto d3_pose = frame.frenetToCartesian(d3.frenet.s, d3.frenet.d);
  d3.x = d3_pose.x;
  d3.y = d3_pose.y;

  overtake_planner::BlockedInfo blocked;
  blocked.pass_proposal_acceleration_allowed = true;
  const auto profile = profile_builder.build(
      10.0, ego, d3, {d3}, overtake_planner::CandidateType::PASS_LEFT, blocked);

  ASSERT_TRUE(profile.has_value());
  EXPECT_NEAR(profile->start_d_m, ego.frenet.d, 1.0e-9);

  overtake_planner::CandidateBuilder candidate_builder(frame, config);
  auto candidate = candidate_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {d3},
      &*profile, true,
      overtake_planner::CandidatePurpose::PROPOSAL_SAFETY_EVALUATION);
  const auto execution_candidate = candidate_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {d3},
      &*profile, true);
  EXPECT_TRUE(candidate.pass_target_corridor_valid);
  EXPECT_TRUE(candidate.desired_path_trackable);
  EXPECT_TRUE(candidate.pure_pursuit_command_trackable);
  EXPECT_TRUE(candidate.controller_tracking_profile_valid);
  ASSERT_FALSE(candidate.longitudinal_offsets_m.empty());
  EXPECT_GE(candidate.longitudinal_offsets_m.back(),
            candidate.required_controller_spatial_horizon_m);
  ASSERT_FALSE(candidate.predicted_speed_mps.empty());
  ASSERT_FALSE(execution_candidate.predicted_speed_mps.empty());
  EXPECT_GT(candidate.predicted_speed_mps.back(), ego.v);
  EXPECT_NEAR(execution_candidate.predicted_speed_mps.back(), ego.v, 1.0e-9);
  ASSERT_FALSE(candidate.x.empty());
  ASSERT_FALSE(candidate.y.empty());
  ASSERT_FALSE(candidate.yaw.empty());
  EXPECT_NEAR(candidate.x.front(), ego.x, 1.0e-6);
  EXPECT_NEAR(candidate.y.front(), ego.y, 1.0e-6);
  EXPECT_NEAR(candidate.yaw.front(), ego.yaw, 1.0e-6);

  overtake_planner::PredictedOpponent prediction;
  prediction.id = d3.id;
  for (const double t_sec : candidate.t) {
    const auto point = frame.frenetToCartesian(d3.frenet.s, d3.frenet.d);
    prediction.t.push_back(t_sec);
    prediction.x.push_back(point.x);
    prediction.y.push_back(point.y);
    prediction.s.push_back(d3.frenet.s);
    prediction.d.push_back(d3.frenet.d);
  }
  auto safety_candidate = candidate;
  overtake_planner::SafetyEvaluator evaluator(frame, config);
  EXPECT_FALSE(evaluator.evaluate(safety_candidate, {prediction}));
  EXPECT_TRUE(safety_candidate.safety_evaluated);
  EXPECT_FALSE(safety_candidate.feasible);
  EXPECT_EQ(safety_candidate.reject_reason, "opponent_collision");
  EXPECT_TRUE(safety_candidate.desired_path_trackable);
  EXPECT_TRUE(safety_candidate.pure_pursuit_command_trackable);
  EXPECT_TRUE(safety_candidate.controller_tracking_profile_valid);
}

TEST(OvertakeSupervisorV2,
     LatestD3LargeLateralErrorStillBuildsProposalFromActualPose) {
  // 20260727-124952 D3のlarge_lateral_errorを、CoreのRECOVERY仲裁や
  // motion authorityから分離する。基準線から1.62 m離れていてもproposalは
  // 中心線へsnapせず、実測姿勢から候補を生成する。
  const auto frame = loopFrame();
  overtake_planner::PlannerConfig config;
  config.v_passthrough_mps = 10.0;
  config.pass_speed_cap_mps = 10.0;
  config.opponent_stale_time_sec = 1.0;
  config.input_future_stamp_tolerance_sec = 0.1;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  config.large_lateral_error_threshold_m = 0.60;
  // 始端契約だけを固定するwide fixture。runtime wall/corridorは緩めない。
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;

  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.stamp_sec = 10.0;
  ego.v = 7.8;
  ego.frenet.s = 42.0;
  ego.frenet.d = -1.62;
  const auto ego_pose = frame.frenetToCartesian(ego.frenet.s, ego.frenet.d);
  ego.x = ego_pose.x;
  ego.y = ego_pose.y;
  ego.yaw = ego_pose.yaw;

  overtake_planner::OpponentState stopped;
  stopped.id = "d2";
  stopped.valid = true;
  stopped.stamp_sec = ego.stamp_sec;
  stopped.v = 0.0;
  stopped.vx = 0.0;
  stopped.vy = 0.0;
  stopped.frenet.s = frame.wrapS(ego.frenet.s + 7.65);
  stopped.frenet.d = -0.2;
  const auto stopped_pose =
      frame.frenetToCartesian(stopped.frenet.s, stopped.frenet.d);
  stopped.x = stopped_pose.x;
  stopped.y = stopped_pose.y;

  overtake_planner::BlockedInfo blocked;
  blocked.pass_proposal_acceleration_allowed = true;
  overtake_planner::V2LocalizedPassProfileBuilder profile_builder(frame,
                                                                  config);
  const auto profile = profile_builder.build(
      ego.stamp_sec, ego, stopped, {stopped},
      overtake_planner::CandidateType::PASS_RIGHT, blocked);
  ASSERT_TRUE(profile.has_value());
  EXPECT_NEAR(profile->anchor_s_m, ego.frenet.s, 1.0e-9);
  EXPECT_NEAR(profile->start_d_m, ego.frenet.d, 1.0e-9);

  overtake_planner::CandidateBuilder candidate_builder(frame, config);
  const auto proposal = candidate_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {stopped},
      &*profile, true,
      overtake_planner::CandidatePurpose::PROPOSAL_SAFETY_EVALUATION);
  ASSERT_FALSE(proposal.x.empty());
  ASSERT_FALSE(proposal.y.empty());
  ASSERT_FALSE(proposal.yaw.empty());
  ASSERT_FALSE(proposal.d.empty());
  EXPECT_NEAR(proposal.x.front(), ego.x, 1.0e-6);
  EXPECT_NEAR(proposal.y.front(), ego.y, 1.0e-6);
  EXPECT_NEAR(proposal.yaw.front(), ego.yaw, 1.0e-6);
  EXPECT_NEAR(proposal.d.front(), ego.frenet.d, 1.0e-9);
}

TEST(OvertakeSupervisorV2,
     EarlyLowSpeedProfileStartsNowAndReachesOffsetBeforeTargetClearance) {
  const auto frame = loopFrame();
  auto config = proposalConfig();
  config.horizon_points = 160;
  config.lateral_override_max_evaluation_horizon_sec = 20.0;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  config.safety_ellipse_a_m = 3.0;
  config.min_ellipse_h = 0.2;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;

  auto ego = proposalEgo(frame);
  ego.v = 6.8;
  ego.frenet.d = 0.0;
  const auto ego_pose = frame.frenetToCartesian(ego.frenet.s, ego.frenet.d);
  ego.x = ego_pose.x;
  ego.y = ego_pose.y;
  ego.yaw = ego_pose.yaw;
  overtake_planner::OpponentState stopped;
  stopped.id = "d3";
  stopped.valid = true;
  stopped.stamp_sec = ego.stamp_sec;
  stopped.frenet.s = frame.wrapS(ego.frenet.s + 30.0);
  stopped.frenet.d = 0.0;
  stopped.v = 0.0;
  stopped.vx = 0.0;
  stopped.vy = 0.0;
  const auto stopped_pose =
      frame.frenetToCartesian(stopped.frenet.s, stopped.frenet.d);
  stopped.x = stopped_pose.x;
  stopped.y = stopped_pose.y;

  overtake_planner::BlockedInfo blocked;
  blocked.opponent_prediction_inputs_complete = true;
  blocked.pass_proposal_acceleration_allowed = true;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_target_index = 0;
  blocked.maneuver_target_id = stopped.id;
  blocked.early_low_speed_pass_target_active = true;
  blocked.early_low_speed_pass_target_index = 0;
  blocked.early_low_speed_pass_target_id = stopped.id;
  blocked.early_low_speed_pass_target_delta_s_m = 30.0;
  overtake_planner::CandidateBuilder candidate_builder(frame, config);
  blocked.early_low_speed_pass_required_transition_m =
      candidate_builder.minimumTrackableLateralShiftDistance(
          ego, -1.0, config.pass_speed_cap_mps, blocked, 4.0);
  blocked.early_low_speed_pass_required_controller_arc_m =
      candidate_builder.requiredControllerSpatialHorizon(
          ego.v, config.pass_speed_cap_mps);

  overtake_planner::V2LocalizedPassProfileBuilder profile_builder(frame,
                                                                  config);
  const auto profile = profile_builder.build(
      ego.stamp_sec, ego, stopped, {stopped},
      overtake_planner::CandidateType::PASS_LEFT, blocked);
  ASSERT_TRUE(profile.has_value());
  EXPECT_NEAR(profile->avoid_start_s_m, profile->anchor_s_m, 1.0e-6);
  const double longitudinal_clearance_m =
      config.safety_ellipse_a_m * std::sqrt(1.0 + config.min_ellipse_h);
  EXPECT_LE(profile->full_offset_start_s_m,
            profile->target_s_m - longitudinal_clearance_m + 1.0e-6);

  const auto proposal = candidate_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {stopped},
      &*profile, true,
      overtake_planner::CandidatePurpose::PROPOSAL_SAFETY_EVALUATION);
  EXPECT_TRUE(proposal.pass_transition_deadline.evaluated);
  EXPECT_TRUE(proposal.pass_transition_deadline.reachable);
  EXPECT_GE(proposal.longitudinal_offsets_m.back(),
            proposal.required_controller_spatial_horizon_m);
}

TEST(OvertakeSupervisorV2,
     LatestDev3D1D2StartCurrentDFollowExposesFirstCandidateFailure) {
  // 20260726-173845 D1の最初のarmed周期を固定する。D1はd3を前方target、
  // d2を右後方のparallel車として観測し、左右PASSが不成立だった。
  // ここではCore仲裁を混ぜず、同じd3を追うcurrent-d FOLLOWを
  // CandidateBuilderと全相手SafetyEvaluatorへ順に通して最初のfalseを固定する。
  overtake_planner::FrenetFrame frame;
  const std::string source_dir = OVERTAKE_PLANNER_SOURCE_DIR;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      source_dir +
          "/../multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv",
      &error))
      << error;
  ASSERT_TRUE(frame.loadCorridorCsv(
      source_dir + "/config/final_ver3_drivable_corridor.csv", &error))
      << error;
  overtake_planner::PlannerConfig config;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.v_passthrough_mps = 50.0;
  config.d_min_m = -1.35;
  config.d_max_m = 1.35;
  config.min_wall_margin_m = 0.25;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 1.554;
  config.ego_rear_extent_m = 0.510;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  config.wall_footprint_max_sample_distance_m = 0.250;
  config.wall_footprint_max_sample_yaw_rad = 0.050;
  config.follow_gap_closing_enabled = true;
  config.follow_speed_margin_mps = 0.0;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.80;
  config.start_grid_attack_follow_v_max_mps = 3.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.lateral_override_max_evaluation_horizon_sec = 20.0;
  config.moving_lateral_override_max_evaluation_horizon_sec = 6.0;
  config.attack_follow_max_evaluation_horizon_sec = 4.0;
  config.attack_follow_min_spatial_horizon_m = 0.55;
  config.attack_follow_max_steering_angle_rad = 0.64;
  config.attack_follow_max_steering_rate_radps = 128.0;
  config.attack_follow_steering_tire_angle_gain = 1.54;
  config.attack_follow_steering_rate_reserve_ratio = 0.80;
  config.lateral_override_lookahead_gain = 0.5;
  config.lateral_override_lookahead_min_distance_m = 3.5;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.80;
  config.min_ellipse_h = 0.20;

  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.stamp_sec = 10.0;
  ego.v = 0.0;
  ego.frenet.s = 28.31;
  ego.frenet.d = 2.58;
  const auto ego_pose = frame.frenetToCartesian(ego.frenet.s, ego.frenet.d);
  ego.x = ego_pose.x;
  ego.y = ego_pose.y;
  ego.yaw = ego_pose.yaw;

  const auto make_stationary_opponent = [&frame, &ego](const std::string &id,
                                                       double delta_s_m,
                                                       double delta_d_m) {
    overtake_planner::OpponentState opponent;
    opponent.id = id;
    opponent.valid = true;
    opponent.stamp_sec = ego.stamp_sec;
    opponent.v = 0.0;
    opponent.vx = 0.0;
    opponent.vy = 0.0;
    opponent.frenet.s = ego.frenet.s + delta_s_m;
    opponent.frenet.d = ego.frenet.d + delta_d_m;
    const auto pose =
        frame.frenetToCartesian(opponent.frenet.s, opponent.frenet.d);
    opponent.x = pose.x;
    opponent.y = pose.y;
    return opponent;
  };
  const auto d3 = make_stationary_opponent("d3", 4.29, -0.82);
  const auto d2 = make_stationary_opponent("d2", 1.10, -3.58);
  const std::vector<overtake_planner::OpponentState> opponents{d3, d2};

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.nearest_index = 0;
  blocked.nearest_id = d3.id;
  blocked.start_grid_target_active = true;
  blocked.start_grid_target_index = 0;
  blocked.start_grid_target_id = d3.id;
  blocked.start_grid_target_delta_s = 4.29;
  blocked.start_grid_target_speed_mps = 0.0;
  blocked.prestart_attack_follow_hold_lateral = true;
  blocked.follow_gap_closing_allowed = true;
  blocked.opponent_prediction_inputs_complete = true;

  overtake_planner::CandidateBuilder builder(frame, config);
  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, opponents);

  ASSERT_FALSE(candidate.d.empty());
  EXPECT_TRUE(
      std::all_of(candidate.d.begin(), candidate.d.end(), [&ego](double d_m) {
        return std::abs(d_m - ego.frenet.d) <= 1.0e-9;
      }));
  EXPECT_TRUE(candidate.longitudinal_profile_valid);
  EXPECT_TRUE(candidate.pass_target_corridor_valid);
  EXPECT_TRUE(candidate.desired_path_trackable);
  EXPECT_TRUE(candidate.pure_pursuit_command_trackable);
  EXPECT_FALSE(candidate.controller_tracking_profile_valid);
  EXPECT_TRUE(std::all_of(
      candidate.v_ref.begin(), candidate.v_ref.end(),
      [](double speed_mps) { return std::abs(speed_mps) <= 1.0e-9; }));
  ASSERT_FALSE(candidate.predicted_speed_mps.empty());
  EXPECT_GT(candidate.predicted_speed_mps.front(), 0.0);
  EXPECT_NEAR(candidate.predicted_speed_mps.back(), 0.0, 1.0e-9);
  const double longitudinal_clearance_m =
      config.safety_ellipse_a_m * std::sqrt(1.0 + config.min_ellipse_h);
  const double available_stop_distance_m =
      blocked.start_grid_target_delta_s - longitudinal_clearance_m;
  ASSERT_FALSE(candidate.longitudinal_offsets_m.empty());
  EXPECT_GT(candidate.longitudinal_offsets_m.back(), 0.0);
  EXPECT_LT(candidate.longitudinal_offsets_m.back(), available_stop_distance_m);
  EXPECT_LT(candidate.longitudinal_offsets_m.back(),
            candidate.required_controller_spatial_horizon_m);

  std::vector<overtake_planner::PredictedOpponent> predictions;
  for (const auto &opponent : opponents) {
    overtake_planner::PredictedOpponent prediction;
    prediction.id = opponent.id;
    for (const double t_sec : candidate.t) {
      const auto point =
          frame.frenetToCartesian(opponent.frenet.s, opponent.frenet.d);
      prediction.t.push_back(t_sec);
      prediction.x.push_back(point.x);
      prediction.y.push_back(point.y);
      prediction.s.push_back(opponent.frenet.s);
      prediction.d.push_back(opponent.frenet.d);
    }
    predictions.push_back(std::move(prediction));
  }

  auto evaluated = candidate;
  overtake_planner::SafetyEvaluator evaluator(frame, config);
  EXPECT_FALSE(evaluator.evaluate(evaluated, predictions));
  EXPECT_TRUE(evaluated.safety_evaluated);
  EXPECT_FALSE(evaluated.feasible);
  EXPECT_EQ(evaluated.reject_reason, "untrackable_lateral_profile");
  EXPECT_TRUE(evaluated.blocking_opponent_id.empty());
}

TEST(OvertakeSupervisorV2,
     LatestDev3D1D2StationaryPassExposesTransitionDeadlineDecomposition) {
  // 20260726-203927の最初のPASS deadline不成立を、Core/StateMachine/ACKを
  // 混ぜずCandidateBuilderへ固定する。D1はD3左、D2はD3右の候補で、
  // required transitionがtarget安全楕円までのavailable arcを超えていた。
  overtake_planner::FrenetFrame frame;
  const std::string source_dir = OVERTAKE_PLANNER_SOURCE_DIR;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      source_dir +
          "/../multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv",
      &error))
      << error;
  ASSERT_TRUE(frame.loadCorridorCsv(
      source_dir + "/config/final_ver3_drivable_corridor.csv", &error))
      << error;

  overtake_planner::PlannerConfig config;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.lateral_override_max_evaluation_horizon_sec = 20.0;
  config.moving_lateral_override_max_evaluation_horizon_sec = 6.0;
  config.v_passthrough_mps = 50.0;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.lateral_override_execution_speed_reserve_sec = 0.25;
  config.d_min_m = -1.35;
  config.d_max_m = 1.35;
  config.min_wall_margin_m = 0.25;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.80;
  config.min_ellipse_h = 0.20;
  config.pass_target_lateral_margin_m = 0.10;
  config.prepare_distance_m = 8.0;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.attack_follow_tracking_wheelbase_m = 1.087;
  config.attack_follow_max_steering_angle_rad = 0.64;
  config.attack_follow_max_steering_rate_radps = 128.0;
  config.attack_follow_steering_tire_angle_gain = 1.54;
  config.attack_follow_steering_rate_reserve_ratio = 0.80;
  config.lateral_override_lookahead_gain = 0.5;
  config.lateral_override_lookahead_min_distance_m = 3.5;

  const auto evaluate_layout = [&](double ego_s_m, double ego_d_m,
                                   double target_delta_s_m,
                                   double target_delta_d_m,
                                   overtake_planner::CandidateType pass_type) {
    overtake_planner::EgoState ego;
    ego.valid = true;
    ego.stamp_sec = 10.0;
    ego.v = 0.0;
    ego.frenet.s = ego_s_m;
    ego.frenet.d = ego_d_m;
    const auto ego_pose = frame.frenetToCartesian(ego.frenet.s, ego.frenet.d);
    ego.x = ego_pose.x;
    ego.y = ego_pose.y;
    ego.yaw = ego_pose.yaw;

    overtake_planner::OpponentState target;
    target.id = "d3";
    target.valid = true;
    target.stamp_sec = ego.stamp_sec;
    target.v = 0.0;
    target.vx = 0.0;
    target.vy = 0.0;
    target.frenet.s = frame.wrapS(ego_s_m + target_delta_s_m);
    target.frenet.d = ego_d_m + target_delta_d_m;
    const auto target_pose =
        frame.frenetToCartesian(target.frenet.s, target.frenet.d);
    target.x = target_pose.x;
    target.y = target_pose.y;

    overtake_planner::BlockedInfo blocked;
    blocked.blocked = true;
    blocked.nearest_index = 0;
    blocked.nearest_id = target.id;
    blocked.front_delta_s = target_delta_s_m;
    blocked.start_grid_target_active = true;
    blocked.start_grid_target_index = 0;
    blocked.start_grid_target_id = target.id;
    blocked.start_grid_target_delta_s = target_delta_s_m;
    blocked.start_grid_target_speed_mps = 0.0;
    blocked.opponent_prediction_inputs_complete = true;
    blocked.pass_proposal_acceleration_allowed = true;

    const double required_gap_m =
        config.safety_ellipse_b_m * std::sqrt(1.0 + config.min_ellipse_h) +
        config.pass_target_lateral_margin_m;
    const double target_d_m =
        pass_type == overtake_planner::CandidateType::PASS_LEFT
            ? target.frenet.d + required_gap_m
            : target.frenet.d - required_gap_m;
    overtake_planner::LocalizedLateralProfile profile;
    profile.active = true;
    profile.target_id = target.id;
    profile.pass_type = pass_type;
    profile.anchor_s_m = ego.frenet.s;
    profile.ego_unwrapped_s_m = ego.frenet.s;
    profile.last_ego_wrapped_s_m = ego.frenet.s;
    profile.avoid_start_s_m = ego.frenet.s;
    profile.full_offset_start_s_m = ego.frenet.s + 6.0;
    profile.full_offset_end_s_m = ego.frenet.s + 8.0;
    profile.merge_end_s_m = ego.frenet.s + 12.0;
    profile.start_d_m = ego.frenet.d;
    profile.target_d_m = target_d_m;

    overtake_planner::CandidateBuilder builder(frame, config);
    return builder.makeCandidate(
        pass_type, ego, blocked, {target}, &profile, false,
        overtake_planner::CandidatePurpose::PROPOSAL_SAFETY_EVALUATION);
  };

  const auto assert_deadline = [&](const overtake_planner::CandidateTrajectory
                                       &candidate,
                                   double expected_start_s_m,
                                   double target_delta_s_m) {
    const auto &diagnostic = candidate.pass_transition_deadline;
    ASSERT_TRUE(diagnostic.evaluated);
    EXPECT_TRUE(diagnostic.input_valid);
    EXPECT_TRUE(diagnostic.requires_new_lateral_transition);
    EXPECT_EQ(diagnostic.source,
              overtake_planner::PassTransitionDeadlineSource::TARGET_CLEARANCE);
    EXPECT_FALSE(diagnostic.reachable);
    EXPECT_FALSE(candidate.pass_transition_deadline_reachable);
    EXPECT_EQ(candidate.reject_reason, "pass_transition_deadline_unreachable");
    EXPECT_TRUE(std::isfinite(diagnostic.lateral_shift_m));
    EXPECT_GT(diagnostic.lateral_shift_m, 0.0);
    EXPECT_NEAR(diagnostic.transition_start_s_m, expected_start_s_m, 1.0e-9);
    const double expected_available_m =
        target_delta_s_m -
        config.safety_ellipse_a_m * std::sqrt(1.0 + config.min_ellipse_h);
    EXPECT_NEAR(diagnostic.available_deadline_m, expected_available_m, 1.0e-9);
    EXPECT_NEAR(diagnostic.transition_end_s_m,
                expected_start_s_m + expected_available_m, 1.0e-9);
    EXPECT_GT(diagnostic.required_transition_m,
              diagnostic.available_deadline_m);
    EXPECT_NEAR(diagnostic.deadline_slack_m,
                diagnostic.available_deadline_m -
                    diagnostic.required_transition_m,
                1.0e-9);
    EXPECT_LT(diagnostic.deadline_slack_m, 0.0);
    EXPECT_TRUE(std::isfinite(diagnostic.evaluated_tracking_speed_mps));
    EXPECT_TRUE(std::isfinite(diagnostic.proposal_speed_cap_mps));
    EXPECT_TRUE(std::isfinite(diagnostic.proposal_horizon_sec));
    EXPECT_TRUE(std::isfinite(diagnostic.proposal_endpoint_arc_m));
    EXPECT_TRUE(std::isfinite(diagnostic.proposal_end_speed_mps));
    EXPECT_TRUE(std::isfinite(diagnostic.pp_required_arc_m));
    EXPECT_NEAR(diagnostic.proposal_horizon_sec, candidate.t.back(), 1.0e-9);
    EXPECT_NEAR(diagnostic.proposal_endpoint_arc_m,
                candidate.longitudinal_offsets_m.back(), 1.0e-9);
    EXPECT_NEAR(diagnostic.pp_required_arc_m,
                candidate.required_controller_spatial_horizon_m, 1.0e-9);
  };

  // D1: ego=(28.31,2.58), D3=(+4.30,-0.82), deadline側はPASS_LEFT。
  const auto d1 = evaluate_layout(28.31, 2.58, 4.30, -0.82,
                                  overtake_planner::CandidateType::PASS_LEFT);
  assert_deadline(d1, 28.31, 4.30);
  ::testing::Test::RecordProperty(
      "d1_transition_required_m",
      std::to_string(d1.pass_transition_deadline.required_transition_m));
  ::testing::Test::RecordProperty(
      "d1_transition_available_m",
      std::to_string(d1.pass_transition_deadline.available_deadline_m));
  ::testing::Test::RecordProperty(
      "d1_transition_slack_m",
      std::to_string(d1.pass_transition_deadline.deadline_slack_m));

  // D2: ego=(34.06,-0.76), D3=(+1.97,+1.49), deadline側はPASS_RIGHT。
  const auto d2 = evaluate_layout(34.06, -0.76, 1.97, 1.49,
                                  overtake_planner::CandidateType::PASS_RIGHT);
  assert_deadline(d2, 34.06, 1.97);
  ::testing::Test::RecordProperty(
      "d2_transition_required_m",
      std::to_string(d2.pass_transition_deadline.required_transition_m));
  ::testing::Test::RecordProperty(
      "d2_transition_available_m",
      std::to_string(d2.pass_transition_deadline.available_deadline_m));
  ::testing::Test::RecordProperty(
      "d2_transition_slack_m",
      std::to_string(d2.pass_transition_deadline.deadline_slack_m));
}

TEST(OvertakeSupervisorV2, Gate2PassBeatsFollowImmediately) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true, 0.0),
      candidate(overtake_planner::CandidateType::PASS_LEFT, true, -1.0)};

  const auto decision = supervisor.update(input(candidates));

  EXPECT_EQ(decision.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(decision.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(decision.pass_direction, 1);
  EXPECT_TRUE(decision.trajectory_authorized);
}

TEST(OvertakeSupervisorV2, SideSpecificStartGateCannotAuthorizeWrongSide) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true, 0.0),
      candidate(overtake_planner::CandidateType::PASS_LEFT, true, -2.0),
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true, -1.0)};
  auto gated = input(candidates);
  gated.pass_left_start_allowed = false;
  gated.pass_right_start_allowed = true;

  const auto decision = supervisor.update(gated);

  EXPECT_EQ(decision.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(decision.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(decision.pass_direction, -1);
}

TEST(OvertakeSupervisorV2, RejectedPassSelectsAttackFollow) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true),
      candidate(overtake_planner::CandidateType::PASS_LEFT, false)};

  const auto decision = supervisor.update(input(candidates));

  EXPECT_EQ(decision.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(decision.selected, overtake_planner::CandidateType::FOLLOW);
}

TEST(OvertakeSupervisorV2, SafePassWithoutMatchingProfileCannotEnterPassing) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true),
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  auto missing = input(candidates);
  missing.pass_left_probe.profile = nullptr;

  const auto decision = supervisor.update(missing);

  EXPECT_EQ(decision.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(decision.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(supervisor.passProfile(), nullptr);

  overtake_planner::OvertakeSupervisorV2 mismatched_supervisor;
  auto mismatched_profile =
      passProfile(overtake_planner::CandidateType::PASS_LEFT, "d2", 0.5);
  auto mismatched = input(candidates);
  mismatched.pass_left_probe.profile = &mismatched_profile;
  const auto mismatched_decision = mismatched_supervisor.update(mismatched);
  EXPECT_EQ(mismatched_decision.phase,
            overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(mismatched_supervisor.passProfile(), nullptr);
}

TEST(OvertakeSupervisorV2, Gate2CommitsExactProfileSnapshot) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true)};
  auto provisional =
      passProfile(overtake_planner::CandidateType::PASS_RIGHT, "d2", -1.0);
  auto start = input(candidates);
  start.pass_right_probe = {&candidates.front(), &provisional, "d2"};

  const auto started = supervisor.update(start);
  ASSERT_EQ(started.phase, overtake_planner::TacticalPhase::PASSING);
  ASSERT_NE(supervisor.passProfile(), nullptr);
  EXPECT_NEAR(supervisor.passProfile()->target_d_m, -1.0, 1.0e-9);

  provisional.target_d_m = -2.50;
  provisional.chain_waypoints.front().target_d_m = -2.50;
  const auto continued = supervisor.update(input(candidates));

  EXPECT_EQ(continued.phase, overtake_planner::TacticalPhase::PASSING);
  ASSERT_NE(supervisor.passProfile(), nullptr);
  EXPECT_NEAR(supervisor.passProfile()->target_d_m, -1.0, 1.0e-9);
  EXPECT_TRUE(supervisor.passProfile()->pass_safety_approved_once);
}

TEST(OvertakeSupervisorV2,
     PassProfileProgressAdvancesMarkersButRejectsIdentityMutation) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true)};
  auto start = input(candidates);
  const auto profile =
      passProfile(overtake_planner::CandidateType::PASS_RIGHT, "d2", -1.0);
  start.pass_right_probe = {&candidates.front(), &profile, "d2"};
  ASSERT_EQ(supervisor.update(start).phase,
            overtake_planner::TacticalPhase::PASSING);
  ASSERT_NE(supervisor.passProfile(), nullptr);

  auto progressed = *supervisor.passProfile();
  progressed.ego_unwrapped_s_m = 65.0;
  progressed.last_ego_wrapped_s_m = 5.0;
  progressed.last_ego_stamp_sec = 2.0;
  progressed.target_s_m = 75.0;
  progressed.avoid_start_s_m = 69.0;
  progressed.full_offset_start_s_m = 72.0;
  progressed.full_offset_end_s_m = 78.0;
  progressed.merge_end_s_m = 85.0;
  progressed.chain_waypoints.front().target_s_m = 75.0;
  progressed.chain_waypoints.front().observed_unwrapped_s_m = 75.0;
  progressed.chain_waypoints.front().last_observed_wrapped_s_m = 15.0;
  progressed.chain_waypoints.front().last_observed_stamp_sec = 2.0;
  progressed.chain_tail_s_m = 75.0;

  ASSERT_TRUE(supervisor.applyPassProfileProgress(progressed));
  ASSERT_NE(supervisor.passProfile(), nullptr);
  EXPECT_NEAR(supervisor.passProfile()->ego_unwrapped_s_m, 65.0, 1.0e-9);
  EXPECT_NEAR(supervisor.passProfile()->target_s_m, 75.0, 1.0e-9);
  EXPECT_EQ(supervisor.passProfile()->target_id, "d2");
  EXPECT_EQ(supervisor.passType(), overtake_planner::CandidateType::PASS_RIGHT);

  auto changed_target = progressed;
  changed_target.target_id = "d3";
  EXPECT_FALSE(supervisor.applyPassProfileProgress(changed_target));

  auto changed_side = progressed;
  changed_side.pass_type = overtake_planner::CandidateType::PASS_LEFT;
  EXPECT_FALSE(supervisor.applyPassProfileProgress(changed_side));

  auto changed_lateral_target = progressed;
  changed_lateral_target.target_d_m = -1.2;
  EXPECT_FALSE(supervisor.applyPassProfileProgress(changed_lateral_target));

  auto changed_waypoint = progressed;
  changed_waypoint.chain_waypoints.front().target_id = "d3";
  EXPECT_FALSE(supervisor.applyPassProfileProgress(changed_waypoint));
  ASSERT_NE(supervisor.passProfile(), nullptr);
  EXPECT_EQ(supervisor.passProfile()->target_id, "d2");
  EXPECT_NEAR(supervisor.passProfile()->target_d_m, -1.0, 1.0e-9);
  EXPECT_NEAR(supervisor.passProfile()->target_s_m, 75.0, 1.0e-9);
}

TEST(OvertakeSupervisorV2,
     ProfileBuilderAccumulatesWrapAndHalfLapProgressAtomically) {
  const auto frame = loopFrame();
  overtake_planner::PlannerConfig config;
  config.v_passthrough_mps = 6.0;
  config.opponent_stale_time_sec = 1.5;
  config.input_future_stamp_tolerance_sec = 0.1;
  config.maneuver_latch_target_update_alpha = 1.0;
  config.localized_avoidance_hold_after_target_m = 3.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  overtake_planner::V2LocalizedPassProfileBuilder builder(frame, config);

  auto profile =
      passProfile(overtake_planner::CandidateType::PASS_RIGHT, "d2", -1.0);
  profile.anchor_s_m = 35.0;
  profile.ego_unwrapped_s_m = 35.0;
  profile.last_ego_wrapped_s_m = 35.0;
  profile.last_ego_stamp_sec = 1.0;
  profile.target_s_m = 38.0;
  profile.avoid_start_s_m = 32.0;
  profile.full_offset_start_s_m = 35.0;
  profile.full_offset_end_s_m = 41.0;
  profile.merge_end_s_m = 47.0;
  profile.chain_waypoints = {{"d2", 38.0, -1.0, 38.0, 38.0, 1.0}};
  profile.chain_tail_id = "d2";
  profile.chain_tail_s_m = 38.0;
  profile.chain_tail_speed_mps = 5.0;
  profile.chain_target_count = 1;

  for (int step = 1; step <= 6; ++step) {
    const double stamp_sec = 1.0 + static_cast<double>(step);
    overtake_planner::EgoState ego;
    ego.valid = true;
    ego.stamp_sec = stamp_sec;
    ego.v = 1.0;
    ego.frenet.s = std::fmod(35.0 + static_cast<double>(step), 40.0);
    overtake_planner::OpponentState target;
    target.valid = true;
    target.id = "d2";
    target.stamp_sec = stamp_sec;
    target.v = 5.0;
    target.frenet.s = std::fmod(38.0 + 5.0 * static_cast<double>(step), 40.0);
    ASSERT_TRUE(
        builder.advanceLongitudinalProgress(stamp_sec, ego, {target}, profile));
  }

  EXPECT_NEAR(profile.ego_unwrapped_s_m, 41.0, 1.0e-9);
  EXPECT_NEAR(profile.target_s_m, 68.0, 1.0e-9);
  EXPECT_GT(profile.target_s_m - profile.ego_unwrapped_s_m,
            frame.length() * 0.5);
  EXPECT_EQ(profile.target_id, "d2");
  EXPECT_EQ(profile.pass_type, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NEAR(profile.target_d_m, -1.0, 1.0e-9);

  const auto before_out_of_order = profile;
  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.stamp_sec = 8.0;
  ego.v = 1.0;
  ego.frenet.s = 2.0;
  overtake_planner::OpponentState old_target;
  old_target.valid = true;
  old_target.id = "d2";
  old_target.stamp_sec = 6.0;
  old_target.v = 5.0;
  old_target.frenet.s = 28.0;
  EXPECT_FALSE(
      builder.advanceLongitudinalProgress(8.0, ego, {old_target}, profile));
  EXPECT_NEAR(profile.ego_unwrapped_s_m, before_out_of_order.ego_unwrapped_s_m,
              1.0e-9);
  EXPECT_NEAR(profile.target_s_m, before_out_of_order.target_s_m, 1.0e-9);
}

TEST(OvertakeSupervisorV2,
     ClassifierTargetChangeBeforeGeometricClearEntersAbortHold) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> initial = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  ASSERT_EQ(supervisor.update(input(initial)).phase,
            overtake_planner::TacticalPhase::PASSING);

  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {0.8, 0.8});
  const std::vector<overtake_planner::CandidateTrajectory> changed = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto changed_input = input(changed);
  changed_input.target_vehicle_id = "d3";
  changed_input.current_target_pass_complete = false;
  changed_input.abort_hold_candidate = &hold;
  const auto aborted = supervisor.update(changed_input);

  EXPECT_EQ(aborted.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(aborted.target_vehicle_id, "d2");
  EXPECT_EQ(aborted.pass_direction, 1);
  ASSERT_NE(supervisor.passProfile(), nullptr);
  EXPECT_EQ(supervisor.passProfile()->target_id, "d2");
}

TEST(OvertakeSupervisorV2,
     CompletedTargetRequiresAttackFollowBeforeNextIndependentGate2) {
  overtake_planner::OvertakeSupervisorV2 supervisor(3, 5, 1);
  const std::vector<overtake_planner::CandidateTrajectory> first = {
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true)};
  const auto started = supervisor.update(input(first));
  ASSERT_EQ(started.phase, overtake_planner::TacticalPhase::PASSING);

  auto completion = input(first);
  completion.current_target_pass_complete = true;
  EXPECT_EQ(supervisor.update(completion).phase,
            overtake_planner::TacticalPhase::PASSING);

  const std::vector<overtake_planner::CandidateTrajectory> next = {
      candidate(overtake_planner::CandidateType::FOLLOW, true),
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  auto next_input = input(next);
  next_input.target_vehicle_id = "d3";
  const auto d3_left_profile =
      passProfile(overtake_planner::CandidateType::PASS_LEFT, "d3", 1.0);
  next_input.pass_left_probe = {&next[1], &d3_left_profile, "d3"};
  next_input.pass_right_probe = {};
  const auto handoff = supervisor.update(next_input);

  EXPECT_EQ(handoff.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(handoff.target_vehicle_id, "d3");
  EXPECT_EQ(handoff.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(handoff.reason, "completed_target_recheck_attack_follow");
  EXPECT_EQ(supervisor.passProfile(), nullptr);

  const auto next_pass = supervisor.update(next_input);
  EXPECT_EQ(next_pass.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(next_pass.target_vehicle_id, "d3");
  EXPECT_EQ(next_pass.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_GT(next_pass.attempt_id, started.attempt_id);
}

TEST(OvertakeSupervisorV2, PassCompletionRequiresConsecutiveSafeObservations) {
  overtake_planner::OvertakeSupervisorV2 supervisor(3, 5, 2);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  ASSERT_EQ(supervisor.update(input(pass)).phase,
            overtake_planner::TacticalPhase::PASSING);

  auto completed = input(pass);
  completed.current_target_pass_complete = true;
  EXPECT_EQ(supervisor.update(completed).phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_FALSE(supervisor.passCompletionPending());

  auto noisy = input(pass);
  noisy.current_target_pass_complete = false;
  EXPECT_EQ(supervisor.update(noisy).phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_FALSE(supervisor.passCompletionPending());

  EXPECT_EQ(supervisor.update(completed).phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_FALSE(supervisor.passCompletionPending());
  EXPECT_EQ(supervisor.update(completed).phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_TRUE(supervisor.passCompletionPending());
}

TEST(OvertakeSupervisorV2,
     CompletedTargetSafetyLossHoldsThenRechecksNextTargetWithoutAbort) {
  overtake_planner::OvertakeSupervisorV2 supervisor(3, 5, 2);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true)};
  const auto started = supervisor.update(input(pass));
  ASSERT_EQ(started.phase, overtake_planner::TacticalPhase::PASSING);

  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {-0.8, -0.8});
  auto first_completion = input(pass);
  first_completion.current_target_pass_complete = true;
  first_completion.tracking_usable = false;
  first_completion.authorization_failure_mask =
      overtake_planner::SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE;
  first_completion.abort_hold_candidate = &hold;
  const auto confirming = supervisor.update(first_completion);
  ASSERT_EQ(confirming.phase, overtake_planner::TacticalPhase::PASSING);
  ASSERT_EQ(confirming.reason, "passing_tracking_temporarily_unavailable_hold");
  ASSERT_FALSE(confirming.trajectory_authorized);
  ASSERT_FALSE(supervisor.passCompletionPending());

  auto rejected_pass =
      candidate(overtake_planner::CandidateType::PASS_RIGHT, false);
  rejected_pass.reject_reason = "opponent_collision";
  const std::vector<overtake_planner::CandidateTrajectory> rejected = {
      rejected_pass};
  auto final_completion = input(rejected);
  final_completion.current_target_pass_complete = true;
  final_completion.abort_hold_candidate = &hold;
  const auto pending = supervisor.update(final_completion);

  EXPECT_EQ(pending.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(pending.reason, "passing_complete_pending_target_recheck");
  EXPECT_EQ(pending.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_EQ(pending.target_vehicle_id, "d2");
  EXPECT_EQ(pending.pass_direction, -1);
  EXPECT_EQ(pending.attempt_id, started.attempt_id);
  EXPECT_TRUE(pending.trajectory_authorized);
  ASSERT_TRUE(supervisor.passCompletionPending());
  ASSERT_NE(supervisor.passProfile(), nullptr);
  EXPECT_EQ(supervisor.passProfile()->target_id, "d2");

  const std::vector<overtake_planner::CandidateTrajectory> next = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto next_input = input(next);
  next_input.target_vehicle_id = "d3";
  const auto handoff = supervisor.update(next_input);

  EXPECT_EQ(handoff.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(handoff.reason, "completed_target_recheck_attack_follow");
  EXPECT_EQ(handoff.target_vehicle_id, "d3");
  EXPECT_EQ(handoff.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(supervisor.passProfile(), nullptr);
}

TEST(OvertakeSupervisorV2,
     AttackFollowRetainsTargetUntilCompletedThenHandsOffWithoutFreeRun) {
  overtake_planner::OvertakeSupervisorV2 supervisor(3, 5, 2);
  const std::vector<overtake_planner::CandidateTrajectory> follow = {
      candidate(overtake_planner::CandidateType::FOLLOW, true),
      candidate(overtake_planner::CandidateType::PASS_RIGHT, false)};
  auto start = input(follow);
  start.pass_right_start_allowed = false;
  start.pass_right_probe = {};
  const auto attacking = supervisor.update(start);
  ASSERT_EQ(attacking.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  ASSERT_EQ(attacking.target_vehicle_id, "d2");

  auto first_completion = start;
  first_completion.current_target_pass_complete = true;
  const auto confirming = supervisor.update(first_completion);
  EXPECT_EQ(confirming.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(confirming.target_vehicle_id, "d2");
  EXPECT_FALSE(supervisor.passCompletionPending());

  auto second_completion = first_completion;
  const auto pending = supervisor.update(second_completion);
  EXPECT_EQ(pending.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(pending.reason, "attack_follow_complete_pending_target_recheck");
  EXPECT_EQ(pending.target_vehicle_id, "d2");
  EXPECT_TRUE(supervisor.passCompletionPending());

  auto next = start;
  next.target_vehicle_id = "d3";
  const auto handed_off = supervisor.update(next);
  EXPECT_EQ(handed_off.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(handed_off.reason, "completed_attack_follow_target_handoff");
  EXPECT_EQ(handed_off.target_vehicle_id, "d3");
  EXPECT_EQ(handed_off.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_FALSE(supervisor.passCompletionPending());
}

TEST(OvertakeSupervisorV2,
     AttackFollowTargetDropoutKeepsIdentityWithBoundedFailClosedHold) {
  overtake_planner::OvertakeSupervisorV2 supervisor(3, 5, 2, 2);
  const std::vector<overtake_planner::CandidateTrajectory> follow = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto start = input(follow);
  start.pass_left_start_allowed = false;
  start.pass_right_start_allowed = false;
  start.pass_left_probe = {};
  ASSERT_EQ(supervisor.update(start).phase,
            overtake_planner::TacticalPhase::ATTACK_FOLLOW);

  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {-0.8, -0.8});
  const std::vector<overtake_planner::CandidateTrajectory> empty;
  auto missing = input(empty);
  missing.target_present = false;
  missing.safety_inputs_complete = false;
  missing.latched_target_temporarily_unavailable = true;
  missing.authorization_failure_mask =
      overtake_planner::SUPERVISOR_V2_AUTH_OPPONENT_STALE;
  missing.abort_hold_candidate = &hold;

  const auto first = supervisor.update(missing);
  const auto second = supervisor.update(missing);
  EXPECT_EQ(first.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(second.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(first.target_vehicle_id, "d2");
  EXPECT_EQ(first.reason, "attack_follow_target_temporarily_unavailable_hold");
  EXPECT_FALSE(first.trajectory_authorized);

  const auto timed_out = supervisor.update(missing);
  EXPECT_EQ(timed_out.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(timed_out.target_vehicle_id, "d2");
  EXPECT_EQ(timed_out.reason, "attack_follow_target_unavailable_timeout");
}

TEST(OvertakeSupervisorV2,
     TemporaryTargetLossKeepsTransactionButFailsClosedThenTimesOut) {
  overtake_planner::OvertakeSupervisorV2 supervisor(3, 5, 2, 2);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true)};
  const auto started = supervisor.update(input(pass));
  ASSERT_EQ(started.phase, overtake_planner::TacticalPhase::PASSING);

  const std::vector<overtake_planner::CandidateTrajectory> no_candidates;
  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {-0.8, -0.8});
  auto missing = input(no_candidates);
  missing.target_present = false;
  missing.safety_inputs_complete = false;
  missing.latched_target_temporarily_unavailable = true;
  missing.authorization_failure_mask =
      overtake_planner::SUPERVISOR_V2_AUTH_OPPONENT_STALE;
  missing.abort_hold_candidate = &hold;

  const auto first_missing = supervisor.update(missing);
  const auto second_missing = supervisor.update(missing);
  EXPECT_EQ(first_missing.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(second_missing.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(first_missing.target_vehicle_id, "d2");
  EXPECT_EQ(first_missing.pass_direction, -1);
  EXPECT_EQ(first_missing.reason,
            "passing_target_temporarily_unavailable_hold");
  EXPECT_FALSE(first_missing.trajectory_authorized);
  ASSERT_NE(supervisor.passProfile(), nullptr);
  EXPECT_EQ(supervisor.passProfile()->target_id, "d2");

  const auto recovered = supervisor.update(input(pass));
  EXPECT_EQ(recovered.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(recovered.target_vehicle_id, "d2");
  EXPECT_EQ(recovered.attempt_id, started.attempt_id);

  EXPECT_EQ(supervisor.update(missing).phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(supervisor.update(missing).phase,
            overtake_planner::TacticalPhase::PASSING);
  const auto timed_out = supervisor.update(missing);
  EXPECT_EQ(timed_out.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(timed_out.reason, "passing_target_unavailable_timeout");
  EXPECT_EQ(timed_out.attempt_id, started.attempt_id);
}

TEST(OvertakeSupervisorV2,
     TemporaryTrackingMismatchHoldsTransactionThenResumesSamePass) {
  overtake_planner::OvertakeSupervisorV2 supervisor(3, 5, 2, 2, 2);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true)};
  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {-0.8, -0.8});
  const auto started = supervisor.update(input(pass));
  ASSERT_EQ(started.phase, overtake_planner::TacticalPhase::PASSING);

  auto mismatch = input(pass);
  mismatch.tracking_usable = false;
  mismatch.authorization_failure_mask =
      overtake_planner::SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE;
  mismatch.abort_hold_candidate = &hold;
  const auto held = supervisor.update(mismatch);

  EXPECT_EQ(held.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_EQ(held.reason, "passing_tracking_temporarily_unavailable_hold");
  EXPECT_EQ(held.target_vehicle_id, "d2");
  EXPECT_EQ(held.pass_direction, -1);
  EXPECT_EQ(held.attempt_id, started.attempt_id);
  EXPECT_FALSE(held.trajectory_authorized);
  ASSERT_NE(supervisor.passProfile(), nullptr);
  EXPECT_EQ(supervisor.passProfile()->target_id, "d2");

  const auto resumed = supervisor.update(input(pass));
  EXPECT_EQ(resumed.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(resumed.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(resumed.target_vehicle_id, "d2");
  EXPECT_EQ(resumed.pass_direction, -1);
  EXPECT_EQ(resumed.attempt_id, started.attempt_id);
  EXPECT_EQ(resumed.reason, "passing_same_generation_side");
}

TEST(OvertakeSupervisorV2,
     ConsecutiveTrackingFailureTimesOutButMpcHardFailureAbortsImmediately) {
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {0.8, 0.8});
  overtake_planner::OvertakeSupervisorV2 timeout_supervisor(3, 5, 2, 2, 1);
  const auto started = timeout_supervisor.update(input(pass));
  auto mismatch = input(pass);
  mismatch.tracking_usable = false;
  mismatch.authorization_failure_mask =
      overtake_planner::SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE;
  mismatch.abort_hold_candidate = &hold;

  EXPECT_EQ(timeout_supervisor.update(mismatch).phase,
            overtake_planner::TacticalPhase::PASSING);
  const auto timed_out = timeout_supervisor.update(mismatch);
  EXPECT_EQ(timed_out.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(timed_out.reason, "passing_tracking_unavailable_timeout");
  EXPECT_EQ(timed_out.attempt_id, started.attempt_id);

  overtake_planner::OvertakeSupervisorV2 hard_failure_supervisor;
  ASSERT_EQ(hard_failure_supervisor.update(input(pass)).phase,
            overtake_planner::TacticalPhase::PASSING);
  auto hard_failure = input(pass);
  hard_failure.tracking_usable = false;
  hard_failure.authorization_failure_mask =
      overtake_planner::SUPERVISOR_V2_AUTH_MPC_HARD_FAILURE;
  hard_failure.abort_hold_candidate = &hold;
  const auto aborted = hard_failure_supervisor.update(hard_failure);
  EXPECT_EQ(aborted.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(aborted.reason, "passing_safety_or_freshness_lost");
}

TEST(OvertakeSupervisorV2, PassingDoesNotFlipSideAndLossEntersAbortHold) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> initial = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  const auto passing = supervisor.update(input(initial));
  ASSERT_EQ(passing.phase, overtake_planner::TacticalPhase::PASSING);

  const std::vector<overtake_planner::CandidateTrajectory> changed = {
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true),
      candidate(overtake_planner::CandidateType::RECOVERY, true)};
  auto changed_input = input(changed);
  changed_input.abort_hold_candidate = &changed[1];
  const auto aborted = supervisor.update(changed_input);

  EXPECT_EQ(aborted.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(aborted.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_NE(aborted.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(aborted.pass_direction, 1);
  EXPECT_EQ(aborted.attempt_id, passing.attempt_id);
}

TEST(OvertakeSupervisorV2, PassingDoesNotReapplyStartOnlyGate) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  auto first_input = input(candidates);
  const auto started = supervisor.update(first_input);
  ASSERT_EQ(started.phase, overtake_planner::TacticalPhase::PASSING);

  auto continuation = input(candidates);
  continuation.pass_left_start_allowed = false;
  continuation.pass_right_start_allowed = false;
  const auto continued = supervisor.update(continuation);

  EXPECT_EQ(continued.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(continued.attempt_id, started.attempt_id);
}

TEST(OvertakeSupervisorV2, UnusableTrackingDoesNotAuthorizeTrajectory) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto unusable = input(candidates);
  unusable.tracking_usable = false;

  const auto decision = supervisor.update(unusable);

  EXPECT_EQ(decision.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_FALSE(decision.trajectory_authorized);
  EXPECT_NE(decision.authorization_failure_mask &
                overtake_planner::SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE,
            0U);
}

TEST(OvertakeSupervisorV2, UnevaluatedCandidateCannotBeAuthorized) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  candidates.front().safety_evaluated = false;

  const auto decision = supervisor.update(input(candidates));

  EXPECT_FALSE(decision.trajectory_authorized);
  EXPECT_NE(
      decision.authorization_failure_mask &
          overtake_planner::SUPERVISOR_V2_AUTH_CANDIDATE_NOT_SAFETY_EVALUATED,
      0U);
}

TEST(OvertakeSupervisorV2, AbortHoldRequiresConsecutiveClearCycles) {
  overtake_planner::OvertakeSupervisorV2 supervisor(2, 1);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  supervisor.update(input(pass));

  const std::vector<overtake_planner::CandidateTrajectory> hold = {
      candidate(overtake_planner::CandidateType::RECOVERY, true),
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto hold_input = input(hold);
  hold_input.abort_hold_candidate = &hold[0];
  hold_input.abort_centering_candidate = &hold[0];
  hold_input.abort_centering_safe = true;
  hold_input.abort_centered = true;
  const auto entered = supervisor.update(hold_input);
  const auto still_held = supervisor.update(hold_input);
  const auto released = supervisor.update(hold_input);

  EXPECT_EQ(entered.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(still_held.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(released.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
}

TEST(OvertakeSupervisorV2, AbortUsesCurrentDHoldUntilCenteringGateIsPermitted) {
  overtake_planner::OvertakeSupervisorV2 supervisor(3, 2);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  supervisor.update(input(pass));

  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {0.8, 0.8, 0.8});
  const auto centering = candidateWithD(
      overtake_planner::CandidateType::RECOVERY, true, {0.8, 0.4, 0.0});
  const std::vector<overtake_planner::CandidateTrajectory> fallback = {
      candidate(overtake_planner::CandidateType::FOLLOW, true),
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true)};
  auto abort_input = input(fallback);
  abort_input.abort_hold_candidate = &hold;
  abort_input.abort_centering_candidate = &centering;
  abort_input.abort_centering_safe = true;
  abort_input.abort_centered = false;
  abort_input.abort_release_allowed = false;

  const auto entered = supervisor.update(abort_input);
  const auto pending = supervisor.update(abort_input);
  const auto centering_started = supervisor.update(abort_input);

  EXPECT_EQ(entered.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(entered.trajectory.d, hold.d);
  EXPECT_EQ(pending.trajectory.d, hold.d);
  EXPECT_EQ(centering_started.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(centering_started.trajectory.d, centering.d);
  EXPECT_EQ(centering_started.pass_direction, 1);
  EXPECT_NE(centering_started.trajectory.d, fallback[1].d);
}

TEST(OvertakeSupervisorV2,
     AbortCenteringFallsBackToCurrentDHoldOnStaleOrReject) {
  overtake_planner::OvertakeSupervisorV2 supervisor(3, 1);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  supervisor.update(input(pass));

  auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY, true,
                             {0.8, 0.8, 0.8});
  auto centering = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                  true, {0.8, 0.4, 0.0});
  auto stop = candidateWithD(overtake_planner::CandidateType::SAFE_STOP, true,
                             {0.8, 0.8, 0.8});
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto abort_input = input(candidates);
  abort_input.abort_hold_candidate = &hold;
  abort_input.abort_centering_candidate = &centering;
  abort_input.abort_stop_candidate = &stop;
  abort_input.abort_centering_safe = true;
  abort_input.abort_centered = false;
  abort_input.abort_release_allowed = false;
  supervisor.update(abort_input);
  const auto centering_started = supervisor.update(abort_input);
  ASSERT_EQ(centering_started.trajectory.d, centering.d);

  auto stale = abort_input;
  stale.safety_inputs_complete = false;
  stale.abort_centering_safe = false;
  const auto stale_hold = supervisor.update(stale);
  EXPECT_EQ(stale_hold.trajectory.d, hold.d);
  EXPECT_FALSE(stale_hold.trajectory_authorized);

  centering.feasible = false;
  centering.cbf_slack = 0.2;
  abort_input.abort_centering_safe = false;
  const auto rejected_hold = supervisor.update(abort_input);
  EXPECT_EQ(rejected_hold.trajectory.d, hold.d);
  EXPECT_TRUE(rejected_hold.trajectory_authorized);

  hold.feasible = false;
  const auto stopped = supervisor.update(abort_input);
  EXPECT_EQ(stopped.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_EQ(stopped.trajectory.d, stop.d);
}

TEST(OvertakeSupervisorV2,
     AbortReleaseCountsOnlyCenteredFreshCyclesAndResetsOnUnsafeCycle) {
  overtake_planner::OvertakeSupervisorV2 supervisor(2, 1);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  supervisor.update(input(pass));

  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {0.8, 0.8, 0.8});
  const auto centering = candidateWithD(
      overtake_planner::CandidateType::RECOVERY, true, {0.8, 0.4, 0.0});
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto abort_input = input(candidates);
  abort_input.abort_hold_candidate = &hold;
  abort_input.abort_centering_candidate = &centering;
  abort_input.abort_centering_safe = true;
  abort_input.abort_centered = false;
  abort_input.abort_release_allowed = true;
  supervisor.update(abort_input);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(supervisor.update(abort_input).phase,
              overtake_planner::TacticalPhase::ABORT_HOLD);
  }

  abort_input.abort_centered = true;
  EXPECT_EQ(supervisor.update(abort_input).phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  auto stale = abort_input;
  stale.safety_inputs_complete = false;
  stale.abort_centering_safe = false;
  EXPECT_EQ(supervisor.update(stale).phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(supervisor.update(abort_input).phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(supervisor.update(abort_input).phase,
            overtake_planner::TacticalPhase::ATTACK_FOLLOW);
}

TEST(OvertakeSupervisorV2, SkippedCoreCycleResetsAbortCenteringClearCycles) {
  overtake_planner::OvertakeSupervisorV2 supervisor(2, 2);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  auto pass_input = input(pass);
  pass_input.cycle_sequence = 1U;
  supervisor.update(pass_input);

  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {0.8, 0.8, 0.8});
  const auto centering = candidateWithD(
      overtake_planner::CandidateType::RECOVERY, true, {0.8, 0.4, 0.0});
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto abort_input = input(candidates);
  abort_input.abort_hold_candidate = &hold;
  abort_input.abort_centering_candidate = &centering;
  abort_input.abort_centering_safe = true;
  abort_input.abort_centered = false;
  abort_input.abort_release_allowed = false;
  abort_input.cycle_sequence = 2U;
  supervisor.update(abort_input);

  abort_input.cycle_sequence = 3U;
  EXPECT_EQ(supervisor.update(abort_input).trajectory.d, hold.d);
  // sequence=4のCore updateがearly returnした想定。次のV2評価は5になる。
  abort_input.cycle_sequence = 5U;
  EXPECT_EQ(supervisor.update(abort_input).trajectory.d, hold.d);
  abort_input.cycle_sequence = 6U;
  EXPECT_EQ(supervisor.update(abort_input).trajectory.d, centering.d);
}

TEST(OvertakeSupervisorV2,
     SkippedCoreCycleDuringPassingEntersAbortEvenWhenPassRecovers) {
  overtake_planner::OvertakeSupervisorV2 supervisor(2, 1);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  auto pass_input = input(pass);
  pass_input.cycle_sequence = 1U;
  ASSERT_EQ(supervisor.update(pass_input).phase,
            overtake_planner::TacticalPhase::PASSING);

  const auto hold = candidateWithD(overtake_planner::CandidateType::RECOVERY,
                                   true, {0.6, 0.6, 0.6});
  // sequence=2のCore updateがearly returnし、3では同側PASSが再び安全でも、
  // 未評価周期を跨いだPASS継続は禁止して現在d保持へ落とす。
  pass_input.cycle_sequence = 3U;
  pass_input.abort_hold_candidate = &hold;
  const auto aborted = supervisor.update(pass_input);

  EXPECT_EQ(aborted.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(aborted.trajectory.d, hold.d);
  EXPECT_EQ(aborted.reason, "passing_input_cycle_skipped");
  EXPECT_EQ(aborted.pass_direction, 1);
}

TEST(OvertakeSupervisorV2, SafeStopSelectionAlwaysRequiresStopConstraint) {
  overtake_planner::SupervisorV2Decision authorized_stop;
  authorized_stop.trajectory_authorized = true;
  authorized_stop.selected = overtake_planner::CandidateType::SAFE_STOP;
  authorized_stop.trajectory =
      candidate(overtake_planner::CandidateType::SAFE_STOP, true);
  EXPECT_TRUE(overtake_planner::supervisorV2RequiresStop(authorized_stop));

  overtake_planner::SupervisorV2Decision authorized_hold;
  authorized_hold.trajectory_authorized = true;
  authorized_hold.selected = overtake_planner::CandidateType::RECOVERY;
  authorized_hold.trajectory =
      candidate(overtake_planner::CandidateType::RECOVERY, true);
  EXPECT_FALSE(overtake_planner::supervisorV2RequiresStop(authorized_hold));

  authorized_hold.trajectory_authorized = false;
  EXPECT_TRUE(overtake_planner::supervisorV2RequiresStop(authorized_hold));
}

TEST(OvertakeSupervisorV2,
     InvalidTrajectoryCannotBecomeEffectiveAuthorization) {
  overtake_planner::SupervisorV2Decision decision;
  decision.trajectory_authorized = true;
  decision.selected = overtake_planner::CandidateType::FOLLOW;
  decision.trajectory =
      candidate(overtake_planner::CandidateType::FOLLOW, true);
  decision.trajectory.y.pop_back();

  EXPECT_FALSE(overtake_planner::supervisorV2TrajectoryPublishable(decision));
  EXPECT_FALSE(overtake_planner::supervisorV2EffectivelyAuthorized(decision));
  EXPECT_TRUE(overtake_planner::supervisorV2RequiresStop(decision));
}

TEST(OvertakeSupervisorV2, EvaluatedValidTrajectoryCanBeEffectivelyAuthorized) {
  overtake_planner::SupervisorV2Decision decision;
  decision.trajectory_authorized = true;
  decision.selected = overtake_planner::CandidateType::FOLLOW;
  decision.trajectory =
      candidate(overtake_planner::CandidateType::FOLLOW, true);

  EXPECT_TRUE(overtake_planner::supervisorV2TrajectoryPublishable(decision));
  EXPECT_TRUE(overtake_planner::supervisorV2EffectivelyAuthorized(decision));
  EXPECT_FALSE(overtake_planner::supervisorV2RequiresStop(decision));
}

TEST(OvertakeSupervisorV2, RejectedTrajectoryCannotBeEffectivelyAuthorized) {
  overtake_planner::SupervisorV2Decision decision;
  decision.trajectory_authorized = true;
  decision.selected = overtake_planner::CandidateType::FOLLOW;
  decision.trajectory =
      candidate(overtake_planner::CandidateType::FOLLOW, false);

  EXPECT_FALSE(overtake_planner::supervisorV2EffectivelyAuthorized(decision));
  EXPECT_TRUE(overtake_planner::supervisorV2RequiresStop(decision));
}

TEST(OvertakeSupervisorV2, FailureMaskCannotBeEffectivelyAuthorized) {
  overtake_planner::SupervisorV2Decision decision;
  decision.trajectory_authorized = true;
  decision.selected = overtake_planner::CandidateType::FOLLOW;
  decision.trajectory =
      candidate(overtake_planner::CandidateType::FOLLOW, true);
  decision.authorization_failure_mask =
      overtake_planner::SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE;

  EXPECT_FALSE(overtake_planner::supervisorV2EffectivelyAuthorized(decision));
  EXPECT_TRUE(overtake_planner::supervisorV2RequiresStop(decision));
}

TEST(OvertakeSupervisorV2, IdenticalDecisionKeepsGenerationStable) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};

  const auto first = supervisor.update(input(candidates));
  const auto second = supervisor.update(input(candidates));

  EXPECT_EQ(first.plan_generation, second.plan_generation);
}

TEST(OvertakeSupervisorV2, SemanticSourceMutationAdvancesGeneration) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  candidates.front().predicted_speed_mps = {2.0, 2.0};

  const auto first = supervisor.update(input(candidates));
  candidates.front().predicted_speed_mps.back() = 2.1;
  const auto second = supervisor.update(input(candidates));

  EXPECT_EQ(second.plan_generation, first.plan_generation + 1U);
}

} // namespace
