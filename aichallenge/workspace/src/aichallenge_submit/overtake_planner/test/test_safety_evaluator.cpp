#include "overtake_planner/safety_evaluator.hpp"

#include <gtest/gtest.h>

TEST(SafetyEvaluator, RejectsCandidateOutsideWallMargin)
{
  overtake_planner::PlannerConfig config;
  config.d_min_m = -1.0;
  config.d_max_m = 1.0;
  config.min_wall_margin_m = 0.2;
  overtake_planner::SafetyEvaluator evaluator(config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.d = {0.0, 0.9};
  candidate.x = {0.0, 1.0};
  candidate.y = {0.0, 0.0};
  candidate.yaw = {0.0, 0.0};

  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_EQ(candidate.reject_reason, "wall_margin");
}

TEST(SafetyEvaluator, RejectsCandidateInsideOpponentEllipse)
{
  overtake_planner::PlannerConfig config;
  overtake_planner::SafetyEvaluator evaluator(config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.t = {0.4};
  candidate.d = {0.0};
  candidate.x = {0.0};
  candidate.y = {0.0};
  candidate.yaw = {0.0};

  overtake_planner::PredictedOpponent pred;
  pred.id = "d3";
  pred.t = {0.4};
  pred.x = {1.0};
  pred.y = {0.0};

  EXPECT_FALSE(evaluator.evaluate(candidate, {pred}));
  EXPECT_EQ(candidate.reject_reason, "opponent_collision");
  EXPECT_EQ(candidate.blocking_opponent_id, "d3");
  EXPECT_NEAR(candidate.blocking_time_sec, 0.4, 1.0e-9);
}

TEST(SafetyEvaluator, AcceptsClearCandidate)
{
  overtake_planner::PlannerConfig config;
  overtake_planner::SafetyEvaluator evaluator(config);

  overtake_planner::CandidateTrajectory candidate;
  candidate.d = {0.0};
  candidate.x = {0.0};
  candidate.y = {0.0};
  candidate.yaw = {0.0};

  overtake_planner::PredictedOpponent pred;
  pred.x = {10.0};
  pred.y = {0.0};

  EXPECT_TRUE(evaluator.evaluate(candidate, {pred}));
}
