#include "overtake_planner/safety_evaluator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

TEST(SafetyEvaluator, RejectsCandidateOutsideWallMargin) {
  overtake_planner::PlannerConfig config;
  config.d_min_m = -1.0;
  config.d_max_m = 1.0;
  config.min_wall_margin_m = 0.2;
  overtake_planner::SafetyEvaluator evaluator(config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0, 0.1};
  candidate.longitudinal_offsets_m = {0.0, 1.0};
  candidate.s = {0.0, 1.0};
  candidate.d = {0.0, 0.9};
  candidate.x = {0.0, 1.0};
  candidate.y = {0.0, 0.0};
  candidate.yaw = {0.0, 0.0};
  candidate.predicted_speed_mps = {1.0, 1.0};
  candidate.v_ref = {1.0, 1.0};

  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_TRUE(candidate.safety_evaluated);
  EXPECT_EQ(candidate.reject_reason, "wall_margin");
}

TEST(SafetyEvaluator, RejectsCandidateInsideOpponentEllipse) {
  overtake_planner::PlannerConfig config;
  overtake_planner::SafetyEvaluator evaluator(config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.4};
  candidate.longitudinal_offsets_m = {0.0};
  candidate.s = {0.0};
  candidate.d = {0.0};
  candidate.x = {0.0};
  candidate.y = {0.0};
  candidate.yaw = {0.0};
  candidate.predicted_speed_mps = {1.0};
  candidate.v_ref = {1.0};

  overtake_planner::PredictedOpponent pred;
  pred.id = "d3";
  pred.t = {0.4};
  pred.x = {1.0};
  pred.y = {0.0};
  pred.s = {1.0};
  pred.d = {0.0};

  EXPECT_FALSE(evaluator.evaluate(candidate, {pred}));
  EXPECT_TRUE(candidate.safety_evaluated);
  EXPECT_EQ(candidate.reject_reason, "opponent_collision");
  EXPECT_EQ(candidate.blocking_opponent_id, "d3");
  EXPECT_NEAR(candidate.blocking_time_sec, 0.4, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_x_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_y_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_yaw_rad, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_s_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_d_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_opponent_x_m, 1.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_opponent_y_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_opponent_s_m, 1.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_opponent_d_m, 0.0, 1.0e-9);
}

TEST(SafetyEvaluator, AcceptsClearCandidate) {
  overtake_planner::PlannerConfig config;
  overtake_planner::SafetyEvaluator evaluator(config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0};
  candidate.longitudinal_offsets_m = {0.0};
  candidate.s = {0.0};
  candidate.d = {0.0};
  candidate.x = {0.0};
  candidate.y = {0.0};
  candidate.yaw = {0.0};
  candidate.predicted_speed_mps = {1.0};
  candidate.v_ref = {1.0};

  overtake_planner::PredictedOpponent pred;
  pred.t = {0.0};
  pred.x = {10.0};
  pred.y = {0.0};
  pred.s = {10.0};
  pred.d = {0.0};

  EXPECT_TRUE(evaluator.evaluate(candidate, {pred}));
  EXPECT_TRUE(candidate.safety_evaluated);
}

TEST(SafetyEvaluator, ClearsBlockerSampleDiagnosticBeforeNextEvaluation) {
  overtake_planner::PlannerConfig config;
  overtake_planner::SafetyEvaluator evaluator(config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0};
  candidate.longitudinal_offsets_m = {0.0};
  candidate.s = {2.0};
  candidate.d = {0.1};
  candidate.x = {2.0};
  candidate.y = {0.1};
  candidate.yaw = {0.0};
  candidate.predicted_speed_mps = {1.0};
  candidate.v_ref = {1.0};

  overtake_planner::PredictedOpponent pred;
  pred.id = "d3";
  pred.t = candidate.t;
  pred.x = {2.5};
  pred.y = {0.1};
  pred.s = {2.5};
  pred.d = {0.1};

  ASSERT_FALSE(evaluator.evaluate(candidate, {pred}));
  ASSERT_EQ(candidate.blocking_opponent_id, "d3");

  pred.x = {20.0};
  pred.s = {20.0};
  EXPECT_TRUE(evaluator.evaluate(candidate, {pred}));
  EXPECT_TRUE(candidate.blocking_opponent_id.empty());
  EXPECT_TRUE(std::isnan(candidate.blocking_time_sec));
  EXPECT_TRUE(std::isnan(candidate.blocking_candidate_x_m));
  EXPECT_TRUE(std::isnan(candidate.blocking_candidate_y_m));
  EXPECT_TRUE(std::isnan(candidate.blocking_candidate_yaw_rad));
  EXPECT_TRUE(std::isnan(candidate.blocking_candidate_s_m));
  EXPECT_TRUE(std::isnan(candidate.blocking_candidate_d_m));
  EXPECT_TRUE(std::isnan(candidate.blocking_opponent_x_m));
  EXPECT_TRUE(std::isnan(candidate.blocking_opponent_y_m));
  EXPECT_TRUE(std::isnan(candidate.blocking_opponent_s_m));
  EXPECT_TRUE(std::isnan(candidate.blocking_opponent_d_m));
}

TEST(SafetyEvaluator, RejectsObservedGateTwoCornerContactEnvelope) {
  overtake_planner::PlannerConfig config;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.8;
  config.min_ellipse_h = 0.2;
  overtake_planner::SafetyEvaluator evaluator(config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0};
  candidate.longitudinal_offsets_m = {0.0};
  candidate.s = {0.0};
  candidate.d = {0.0};
  candidate.x = {0.0};
  candidate.y = {0.0};
  candidate.yaw = {0.0};
  candidate.predicted_speed_mps = {2.62};
  candidate.v_ref = {2.62};

  overtake_planner::PredictedOpponent pred;
  pred.id = "gate2_d3";
  pred.t = {0.0};
  // 20260718-181454の物理接触直前に自車body座標で観測した相対位置。
  // 車幅だけからb=1.30へ縮めるとh=0.72で通るが、実際の旋回車体は接触した。
  pred.x = {0.87};
  pred.y = {1.66};
  pred.s = {0.87};
  pred.d = {1.66};

  EXPECT_FALSE(evaluator.evaluate(candidate, {pred}));
  EXPECT_EQ(candidate.reject_reason, "opponent_collision");
  EXPECT_EQ(candidate.blocking_opponent_id, "gate2_d3");
  EXPECT_LT(candidate.min_safety_margin, config.min_ellipse_h);
}

TEST(SafetyEvaluator, RejectsCollisionBetweenSparseExtendedHorizonPoints) {
  overtake_planner::PlannerConfig config;
  config.horizon_dt_sec = 0.025;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.2;
  overtake_planner::SafetyEvaluator evaluator(config);

  // 延長ATTACK_FOLLOW相当の粗い2点。両端では相手が楕円外でも、0.05秒で
  // 自車位置を横切る。端点だけの評価を合格根拠にしてはいけない。
  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0, 0.1};
  candidate.longitudinal_offsets_m = {0.0, 0.0};
  candidate.s = {0.0, 0.0};
  candidate.d = {0.0, 0.0};
  candidate.x = {0.0, 0.0};
  candidate.y = {0.0, 0.0};
  candidate.yaw = {0.0, 0.0};
  candidate.predicted_speed_mps = {0.0, 0.0};
  candidate.v_ref = {0.0, 0.0};

  overtake_planner::PredictedOpponent pred;
  pred.id = "crossing_d3";
  pred.t = candidate.t;
  pred.x = {-4.0, 4.0};
  pred.y = {0.0, 0.0};
  pred.s = {-4.0, 4.0};
  pred.d = {0.0, 0.0};

  EXPECT_FALSE(evaluator.evaluate(candidate, {pred}));
  EXPECT_EQ(candidate.reject_reason, "opponent_collision");
  EXPECT_EQ(candidate.blocking_opponent_id, "crossing_d3");
  EXPECT_NEAR(candidate.blocking_time_sec, 0.025, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_x_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_y_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_yaw_rad, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_s_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_candidate_d_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_opponent_x_m, -2.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_opponent_y_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_opponent_s_m, -2.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_opponent_d_m, 0.0, 1.0e-9);
}

TEST(SafetyEvaluator, RejectsInvalidCandidateShapeWithoutIndexingPastEnd) {
  overtake_planner::PlannerConfig config;
  overtake_planner::SafetyEvaluator evaluator(config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0, 0.1};
  candidate.longitudinal_offsets_m = {0.0, 1.0};
  candidate.s = {0.0, 1.0};
  candidate.d = {0.0, 0.1};
  candidate.x = {0.0, 1.0};
  candidate.y = {0.0};
  candidate.yaw = {0.0, 0.0};
  candidate.predicted_speed_mps = {1.0, 1.0};
  candidate.v_ref = {1.0, 1.0};

  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_TRUE(candidate.safety_evaluated);
  EXPECT_EQ(candidate.reject_reason, "invalid_candidate_horizon");
}

TEST(SafetyEvaluator, UsesVerifiedSDependentCorridorWhenAvailable) {
  overtake_planner::FrenetFrame frame;
  frame.setReference({
      {0.0, 0.0, 0.0, 0.0, 0.0, 5.0},
      {10.0, 10.0, 0.0, 0.0, 0.0, 5.0},
  });
  frame.setCorridor({
      {0.0, -3.0, 3.0},
      {10.0, -3.0, 3.0},
  });
  overtake_planner::PlannerConfig config;
  config.d_min_m = -1.35;
  config.d_max_m = 1.35;
  config.min_wall_margin_m = 0.50;
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0};
  candidate.longitudinal_offsets_m = {0.0};
  candidate.s = {2.0};
  candidate.d = {2.1};
  candidate.x = {2.0};
  candidate.y = {2.1};
  candidate.yaw = {0.0};
  candidate.predicted_speed_mps = {1.0};
  candidate.v_ref = {1.0};

  EXPECT_TRUE(evaluator.evaluate(candidate, {}));
}

TEST(SafetyEvaluator, RejectsRotatedFrontCornerOutsideRawLaneletBoundary) {
  overtake_planner::FrenetFrame frame;
  frame.setReference({
      {0.0, 0.0, 0.0, 0.0, 0.0, 5.0},
      {10.0, 10.0, 0.0, 0.0, 0.0, 5.0},
  });
  frame.setCorridor({
      {0.0, -1.5, 1.5},
      {10.0, -1.5, 1.5},
  });
  overtake_planner::PlannerConfig config;
  config.min_wall_margin_m = 0.5;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 1.554;
  config.ego_rear_extent_m = 0.510;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0};
  candidate.longitudinal_offsets_m = {0.0};
  candidate.s = {5.0};
  candidate.d = {0.0};
  candidate.x = {5.0};
  candidate.y = {0.0};
  candidate.yaw = {M_PI_4};
  candidate.predicted_speed_mps = {1.0};
  candidate.v_ref = {1.0};

  // 中心点は±1.0 mの従来安全帯内だが、45度を向いた前cornerは路面端を越える。
  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_EQ(candidate.reject_reason, "wall_footprint_margin");
  EXPECT_LT(candidate.corridor_min_margin_m, 0.0);
  EXPECT_TRUE(candidate.blocking_wall_footprint_valid);
  EXPECT_EQ(candidate.blocking_wall_segment_index, 0);
  EXPECT_NEAR(candidate.blocking_wall_segment_ratio, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_time_sec, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_candidate_x_m, 5.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_candidate_y_m, 0.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_candidate_yaw_rad, M_PI_4, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_candidate_s_m, 5.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_candidate_d_m, 0.0, 1.0e-9);
  EXPECT_EQ(candidate.blocking_wall_corner_index, 0);
  EXPECT_TRUE(std::isfinite(candidate.blocking_wall_corner_x_m));
  EXPECT_GT(candidate.blocking_wall_corner_y_m, 1.5);
  EXPECT_NEAR(candidate.blocking_wall_corner_s_m,
              candidate.blocking_wall_corner_x_m, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_corner_d_m,
              candidate.blocking_wall_corner_y_m, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_corridor_d_min_m, -1.5, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_corridor_d_max_m, 1.5, 1.0e-9);
  EXPECT_LT(candidate.blocking_wall_physical_clearance_m, 0.0);
  EXPECT_NEAR(candidate.blocking_wall_effective_clearance_m,
              candidate.blocking_wall_physical_clearance_m - 0.250, 1.0e-9);

  // 同じobjectを再評価して成立した場合、前周期のwall blocker診断を残さない。
  candidate.yaw = {0.0};
  EXPECT_TRUE(evaluator.evaluate(candidate, {}));
  EXPECT_FALSE(candidate.blocking_wall_footprint_valid);
  EXPECT_EQ(candidate.blocking_wall_segment_index, -1);
  EXPECT_TRUE(std::isnan(candidate.blocking_wall_segment_ratio));
  EXPECT_TRUE(std::isnan(candidate.blocking_wall_time_sec));
  EXPECT_TRUE(std::isnan(candidate.blocking_wall_candidate_x_m));
  EXPECT_TRUE(std::isnan(candidate.blocking_wall_corner_x_m));
  EXPECT_EQ(candidate.blocking_wall_corner_index, -1);
  EXPECT_TRUE(std::isnan(candidate.blocking_wall_effective_clearance_m));
}

TEST(SafetyEvaluator, AcceptsStraightFootprintWithBoundedLocalizationMargin) {
  overtake_planner::FrenetFrame frame;
  frame.setReference({
      {0.0, 0.0, 0.0, 0.0, 0.0, 5.0},
      {10.0, 10.0, 0.0, 0.0, 0.0, 5.0},
  });
  frame.setCorridor({
      {0.0, -1.0, 1.0},
      {10.0, -1.0, 1.0},
  });
  overtake_planner::PlannerConfig config;
  config.min_wall_margin_m = 0.5;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 1.554;
  config.ego_rear_extent_m = 0.510;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0};
  candidate.longitudinal_offsets_m = {0.0};
  candidate.s = {5.0};
  candidate.d = {0.0};
  candidate.x = {5.0};
  candidate.y = {0.0};
  candidate.yaw = {0.0};
  candidate.predicted_speed_mps = {1.0};
  candidate.v_ref = {1.0};

  EXPECT_TRUE(evaluator.evaluate(candidate, {}));
  EXPECT_NEAR(candidate.corridor_min_margin_m, 0.10, 1.0e-9);
}

TEST(SafetyEvaluator, RecordsFirstSweptWallFootprintRejectSample) {
  overtake_planner::FrenetFrame frame;
  frame.setReference({
      {0.0, 0.0, 0.0, 0.0, 0.0, 5.0},
      {5.0, 5.0, 0.0, 0.0, 0.0, 5.0},
      {10.0, 10.0, 0.0, 0.0, 0.0, 5.0},
  });
  frame.setCorridor({
      {0.0, -2.0, 2.0},
      {1.0, -2.0, 2.0},
      {2.0, -2.0, 2.0},
      {3.0, -2.0, 2.0},
      {4.0, -2.0, 2.0},
      {5.0, -0.8, 0.8},
      {6.0, -2.0, 2.0},
      {7.0, -2.0, 2.0},
      {8.0, -2.0, 2.0},
      {9.0, -2.0, 2.0},
      {10.0, -2.0, 2.0},
  });
  overtake_planner::PlannerConfig config;
  config.min_wall_margin_m = 0.1;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 0.0;
  config.ego_rear_extent_m = 0.0;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  config.wall_footprint_max_sample_distance_m = 1.0;
  config.wall_footprint_max_sample_yaw_rad = 0.2;
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.0, 2.0};
  candidate.longitudinal_offsets_m = {0.0, 10.0};
  candidate.s = {0.0, 10.0};
  candidate.d = {0.0, 0.0};
  candidate.x = {0.0, 10.0};
  candidate.y = {0.0, 0.0};
  candidate.yaw = {0.0, 0.0};
  candidate.predicted_speed_mps = {5.0, 5.0};
  candidate.v_ref = {5.0, 5.0};

  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_EQ(candidate.reject_reason, "wall_footprint_margin");
  EXPECT_TRUE(candidate.blocking_wall_footprint_valid);
  EXPECT_EQ(candidate.blocking_wall_segment_index, 0);
  EXPECT_NEAR(candidate.blocking_wall_segment_ratio, 0.4, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_time_sec, 0.8, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_candidate_x_m, 4.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_candidate_s_m, 4.0, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_candidate_d_m, 0.0, 1.0e-9);
  EXPECT_EQ(candidate.blocking_wall_corner_index, 0);
  EXPECT_NEAR(candidate.blocking_wall_corridor_d_min_m, -0.8, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_corridor_d_max_m, 0.8, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_physical_clearance_m, 0.15, 1.0e-9);
  EXPECT_NEAR(candidate.blocking_wall_effective_clearance_m, -0.10, 1.0e-9);
}

TEST(SafetyEvaluator, RuntimeReferenceFootprintFitsRuntimeLaneletCorridor) {
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
  config.min_wall_margin_m = 0.50;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 1.554;
  config.ego_rear_extent_m = 0.510;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  overtake_planner::CandidateTrajectory candidate;
  const auto &reference = frame.reference();
  ASSERT_GT(reference.size(), 2U);
  const double first_s_m = reference.front().s;
  for (std::size_t i = 0U; i < reference.size(); ++i) {
    const auto &point = reference[i];
    candidate.t.push_back(static_cast<double>(i) * 0.1);
    candidate.longitudinal_offsets_m.push_back(point.s - first_s_m);
    candidate.s.push_back(point.s);
    candidate.d.push_back(0.0);
    candidate.x.push_back(point.x);
    candidate.y.push_back(point.y);
    candidate.yaw.push_back(point.yaw);
    candidate.predicted_speed_mps.push_back(std::max(0.0, point.v_ref));
    candidate.v_ref.push_back(std::max(0.0, point.v_ref));
  }

  ASSERT_TRUE(evaluator.evaluate(candidate, {})) << candidate.reject_reason;
  EXPECT_GT(candidate.corridor_min_margin_m, 0.25);
}
