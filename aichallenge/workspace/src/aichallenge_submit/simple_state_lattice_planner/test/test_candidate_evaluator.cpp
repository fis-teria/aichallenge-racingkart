#include "simple_state_lattice_planner/candidate_evaluator.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace simple_state_lattice_planner {
namespace {

Costmap2D openMap() {
  Costmap2D map;
  map.snapshot_id = 1U;
  map.stamp_sec = 10.0;
  map.origin_x_m = -5.0;
  map.origin_y_m = -5.0;
  map.width = 200U;
  map.height = 100U;
  map.cells.assign(map.width * map.height, 0U);
  return map;
}

Candidate straightCandidate() {
  Candidate candidate;
  candidate.snapshot_id = 1U;
  for (int index = 0; index <= 20; ++index) {
    TrajectoryPoint point;
    point.x_m = 0.05 * static_cast<double>(index);
    point.s_m = point.x_m;
    point.speed_mps = 2.0;
    candidate.points.push_back(point);
  }
  return candidate;
}

TEST(CandidateEvaluator, AcceptsFiniteFreeCandidateDeterministically) {
  const auto first = evaluateCandidate(straightCandidate(), openMap());
  const auto second = evaluateCandidate(straightCandidate(), openMap());
  ASSERT_TRUE(first.valid);
  EXPECT_EQ(first.reject_reason, RejectReason::NONE);
  EXPECT_DOUBLE_EQ(first.total_cost, second.total_cost);
}

TEST(CandidateEvaluator, RejectsOccupiedUnknownAndOutOfBounds) {
  auto map = openMap();
  std::size_t obstacle_x = 0U;
  std::size_t obstacle_y = 0U;
  ASSERT_TRUE(map.worldToCell(0.5, 0.0, &obstacle_x, &obstacle_y));
  map.cells[obstacle_y * map.width + obstacle_x] = kOccupiedCost;
  EXPECT_EQ(evaluateCandidate(straightCandidate(), map).reject_reason,
            RejectReason::COSTMAP_OCCUPIED);

  map = openMap();
  ASSERT_TRUE(map.worldToCell(0.5, 0.0, &obstacle_x, &obstacle_y));
  map.cells[obstacle_y * map.width + obstacle_x] = kUnknownCost;
  EXPECT_EQ(evaluateCandidate(straightCandidate(), map).reject_reason,
            RejectReason::COSTMAP_OCCUPIED);

  auto candidate = straightCandidate();
  candidate.points.back().x_m = 1000.0;
  EXPECT_EQ(evaluateCandidate(candidate, openMap()).reject_reason,
            RejectReason::COSTMAP_OCCUPIED);
}

TEST(CandidateEvaluator, RejectsNonFiniteReverseAndTooFewPoints) {
  auto candidate = straightCandidate();
  candidate.points[2].yaw_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(evaluateCandidate(candidate, openMap()).reject_reason,
            RejectReason::NON_FINITE_POINT);

  candidate = straightCandidate();
  candidate.points[2].s_m = candidate.points[1].s_m;
  EXPECT_EQ(evaluateCandidate(candidate, openMap()).reject_reason,
            RejectReason::REVERSE_S);

  candidate.points.resize(1U);
  EXPECT_EQ(evaluateCandidate(candidate, openMap()).reject_reason,
            RejectReason::TOO_FEW_POINTS);
}

TEST(CandidateEvaluator, RejectsCurvatureAndSteeringLimits) {
  auto candidate = straightCandidate();
  candidate.points[2].curvature_radpm = 0.6;
  EXPECT_EQ(evaluateCandidate(candidate, openMap()).reject_reason,
            RejectReason::CURVATURE_LIMIT);

  auto config = CandidateEvaluatorConfig{};
  config.maximum_curvature_radpm = 10.0;
  config.maximum_steering_angle_rad = 0.1;
  candidate = straightCandidate();
  candidate.points[2].curvature_radpm = 0.2;
  EXPECT_EQ(evaluateCandidate(candidate, openMap(), config).reject_reason,
            RejectReason::STEERING_ANGLE_LIMIT);
}

TEST(CandidateEvaluator, SweepsBetweenVerticesAtBoundedSpacing) {
  auto candidate = straightCandidate();
  candidate.points.resize(2U);
  candidate.points[1].x_m = 0.20;
  candidate.points[1].s_m = 0.20;
  auto map = openMap();
  std::size_t obstacle_x = 0U;
  std::size_t obstacle_y = 0U;
  ASSERT_TRUE(map.worldToCell(0.10, 0.0, &obstacle_x, &obstacle_y));
  map.cells[obstacle_y * map.width + obstacle_x] = kOccupiedCost;
  EXPECT_EQ(evaluateCandidate(candidate, map).reject_reason,
            RejectReason::COSTMAP_OCCUPIED);
}

TEST(CandidateEvaluator, RejectsSnapshotNegativeSpeedAndSteeringReversal) {
  auto candidate = straightCandidate();
  candidate.snapshot_id = 2U;
  EXPECT_EQ(evaluateCandidate(candidate, openMap()).reject_reason,
            RejectReason::SNAPSHOT_MISMATCH);

  candidate = straightCandidate();
  candidate.points[2].speed_mps = -0.1;
  EXPECT_EQ(evaluateCandidate(candidate, openMap()).reject_reason,
            RejectReason::NEGATIVE_SPEED);

  candidate = straightCandidate();
  candidate.points[1].curvature_radpm = 0.10;
  candidate.points[2].curvature_radpm = -0.10;
  EXPECT_EQ(evaluateCandidate(candidate, openMap()).reject_reason,
            RejectReason::STEERING_RATE_LIMIT);
}

}  // namespace
}  // namespace simple_state_lattice_planner
