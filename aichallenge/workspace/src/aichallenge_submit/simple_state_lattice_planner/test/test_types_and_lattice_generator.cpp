#include "simple_state_lattice_planner/candidate_evaluator.hpp"
#include "simple_state_lattice_planner/lattice_generator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace simple_state_lattice_planner {
namespace {

ReferenceWindow straightReference() {
  ReferenceWindow reference;
  reference.snapshot_id = 1U;
  reference.stamp_sec = 10.0;
  for (int index = 0; index <= 60; ++index) {
    ReferencePoint point;
    point.x_m = static_cast<double>(index);
    point.s_m = static_cast<double>(index);
    reference.points.push_back(point);
  }
  return reference;
}

ReferenceWindow cornerReference() {
  ReferenceWindow reference;
  reference.snapshot_id = 1U;
  reference.stamp_sec = 10.0;
  constexpr double radius_m = 20.0;
  constexpr int samples = 80;
  for (int index = 0; index <= samples; ++index) {
    const double angle =
        (M_PI_2 * static_cast<double>(index)) / static_cast<double>(samples);
    ReferencePoint point;
    point.x_m = radius_m * std::sin(angle);
    point.y_m = radius_m * (1.0 - std::cos(angle));
    point.yaw_rad = angle;
    point.s_m = radius_m * angle;
    reference.points.push_back(point);
  }
  return reference;
}

ReferenceWindow sparseArcReference(double radius_m) {
  ReferenceWindow reference;
  reference.snapshot_id = 1U;
  reference.stamp_sec = 10.0;
  const std::array<double, 17U> spacings_m{0.83, 1.34, 0.86, 1.78, 0.91, 1.27,
                                           0.76, 1.62, 0.88, 1.41, 0.79, 1.55,
                                           0.84, 1.31, 0.93, 1.47, 1.29};
  double s_m = 0.0;
  for (int index = 0; s_m < 30.0; ++index) {
    const double angle = s_m / radius_m;
    ReferencePoint point;
    point.x_m = radius_m * std::sin(angle);
    point.y_m = radius_m * (1.0 - std::cos(angle));
    point.yaw_rad = angle;
    point.s_m = s_m;
    reference.points.push_back(point);
    s_m += spacings_m[static_cast<std::size_t>(index) % spacings_m.size()];
  }
  return reference;
}

Costmap2D openCostmap() {
  Costmap2D map;
  map.snapshot_id = 1U;
  map.stamp_sec = 10.0;
  map.resolution_m = 0.1;
  map.origin_x_m = -30.0;
  map.origin_y_m = -30.0;
  map.width = 1000U;
  map.height = 1000U;
  map.cells.assign(map.width * map.height, 0U);
  return map;
}

EgoState validEgo() {
  EgoState ego;
  ego.snapshot_id = 1U;
  ego.stamp_sec = 10.0;
  ego.speed_mps = 1.0;
  return ego;
}

EgoState egoOnArc(double radius_m, double s_m) {
  auto ego = validEgo();
  const double angle = s_m / radius_m;
  ego.x_m = radius_m * std::sin(angle);
  ego.y_m = radius_m * (1.0 - std::cos(angle));
  ego.yaw_rad = angle;
  return ego;
}

void expectFinite(const Candidate &candidate) {
  ASSERT_FALSE(candidate.points.empty());
  for (const auto &point : candidate.points) {
    EXPECT_TRUE(std::isfinite(point.x_m));
    EXPECT_TRUE(std::isfinite(point.y_m));
    EXPECT_TRUE(std::isfinite(point.yaw_rad));
    EXPECT_TRUE(std::isfinite(point.curvature_radpm));
    EXPECT_TRUE(std::isfinite(point.s_m));
    EXPECT_TRUE(std::isfinite(point.d_m));
    EXPECT_TRUE(std::isfinite(point.speed_mps));
  }
}

TEST(LatticeGenerator, GeneratesStableSevenCandidatesOnStraight) {
  const LatticeConfig config;
  const auto first =
      generateLatticeCandidates(straightReference(), validEgo(), config, 10.0);
  const auto second =
      generateLatticeCandidates(straightReference(), validEgo(), config, 10.0);
  ASSERT_TRUE(first.valid()) << first.reason;
  ASSERT_TRUE(second.valid()) << second.reason;
  ASSERT_EQ(first.candidates.size(), 7U);
  EXPECT_EQ(first.snapshot_id, 1U);
  ASSERT_EQ(second.candidates.size(), first.candidates.size());
  const std::array<CandidateSide, 7U> expected_sides{
      CandidateSide::CENTER, CandidateSide::LEFT,  CandidateSide::LEFT,
      CandidateSide::LEFT,   CandidateSide::RIGHT, CandidateSide::RIGHT,
      CandidateSide::RIGHT};
  const std::array<double, 7U> expected_transitions_m{10.0, 6.0,  10.0, 14.0,
                                                      6.0,  10.0, 14.0};
  for (std::size_t index = 0U; index < first.candidates.size(); ++index) {
    const auto &candidate = first.candidates[index];
    EXPECT_EQ(candidate.id, index);
    EXPECT_EQ(candidate.side, expected_sides[index]);
    EXPECT_DOUBLE_EQ(candidate.transition_distance_m,
                     expected_transitions_m[index]);
    EXPECT_EQ(second.candidates[index].id, candidate.id);
    EXPECT_EQ(second.candidates[index].side, candidate.side);
    ASSERT_EQ(second.candidates[index].points.size(), candidate.points.size());
    ASSERT_EQ(candidate.points.size(), 401U);
    expectFinite(candidate);
    EXPECT_NEAR(candidate.points.front().s_m, 0.0, 1.0e-12);
    EXPECT_NEAR(candidate.points.back().s_m, 20.0, 1.0e-12);
    EXPECT_NEAR(candidate.points.front().d_m, 0.0, 1.0e-12);
    for (std::size_t point_index = 1U; point_index < candidate.points.size();
         ++point_index) {
      EXPECT_GT(candidate.points[point_index].s_m,
                candidate.points[point_index - 1U].s_m);
    }
    for (std::size_t point_index = 0U; point_index < candidate.points.size();
         ++point_index) {
      EXPECT_DOUBLE_EQ(second.candidates[index].points[point_index].x_m,
                       candidate.points[point_index].x_m);
      EXPECT_DOUBLE_EQ(second.candidates[index].points[point_index].y_m,
                       candidate.points[point_index].y_m);
      EXPECT_DOUBLE_EQ(second.candidates[index].points[point_index].d_m,
                       candidate.points[point_index].d_m);
    }
  }
  EXPECT_EQ(first.candidates[0].side, CandidateSide::CENTER);
  EXPECT_EQ(first.candidates[1].side, CandidateSide::LEFT);
  EXPECT_EQ(first.candidates[4].side, CandidateSide::RIGHT);
  EXPECT_NEAR(first.candidates[0].points.back().d_m, 0.0, 1.0e-12);
  EXPECT_NEAR(first.candidates[1].points.back().d_m, config.pass_offset_m,
              1.0e-12);
  EXPECT_NEAR(first.candidates[2].points.back().d_m, config.pass_offset_m,
              1.0e-12);
  EXPECT_NEAR(first.candidates[3].points.back().d_m, config.pass_offset_m,
              1.0e-12);
  EXPECT_NEAR(first.candidates[4].points.back().d_m, -config.pass_offset_m,
              1.0e-12);
  EXPECT_NEAR(first.candidates[5].points.back().d_m, -config.pass_offset_m,
              1.0e-12);
  EXPECT_NEAR(first.candidates[6].points.back().d_m, -config.pass_offset_m,
              1.0e-12);
}

TEST(LatticeGenerator, GeneratesFiniteCandidatesOnCorner) {
  const auto result = generateLatticeCandidates(cornerReference(), validEgo(),
                                                LatticeConfig{}, 10.0);
  ASSERT_TRUE(result.valid()) << result.reason;
  ASSERT_EQ(result.candidates.size(), 7U);
  for (const auto &candidate : result.candidates) {
    expectFinite(candidate);
  }
}

TEST(LatticeGenerator, ContinuesOutboundShiftWithoutRestartingTransition) {
  auto ego = validEgo();
  ego.y_m = -0.9;
  auto config = LatticeConfig{};
  config.target_speed_mps = 0.1;
  const auto generated =
      generateLatticeCandidates(straightReference(), ego, config, 10.0);
  ASSERT_TRUE(generated.valid()) << generated.reason;
  ASSERT_EQ(generated.candidates.size(), 7U);
  const auto &right_short = generated.candidates[4];
  EXPECT_DOUBLE_EQ(right_short.transition_distance_m, 6.0);
  EXPECT_NEAR(right_short.remaining_transition_distance_m, 3.0, 1.0e-12);
  ASSERT_GT(right_short.points.size(), 60U);
  EXPECT_NEAR(right_short.points.front().d_m, -0.9, 1.0e-12);
  EXPECT_NEAR(right_short.points[60U].d_m, -1.8, 1.0e-12);
  for (std::size_t index = 1U; index < right_short.points.size(); ++index) {
    EXPECT_LE(right_short.points[index].d_m,
              right_short.points[index - 1U].d_m + 1.0e-12);
    EXPECT_GE(right_short.points[index].d_m, -1.8 - 1.0e-12);
  }

  const auto evaluated = evaluateCandidate(right_short, openCostmap());
  EXPECT_TRUE(evaluated.valid);
  EXPECT_EQ(evaluated.reject_reason, RejectReason::NONE);
}

TEST(LatticeGenerator, SparseGentleArcPassesUnchangedHardLimits) {
  const auto generated = generateLatticeCandidates(
      sparseArcReference(100.0), egoOnArc(100.0, 5.0), LatticeConfig{}, 10.0);
  ASSERT_TRUE(generated.valid()) << generated.reason;
  ASSERT_EQ(generated.candidates.size(), 7U);
  const auto evaluated =
      evaluateCandidate(generated.candidates.front(), openCostmap());
  EXPECT_TRUE(evaluated.valid);
  EXPECT_EQ(evaluated.reject_reason, RejectReason::NONE);
  const CandidateEvaluatorConfig limits;
  EXPECT_DOUBLE_EQ(limits.maximum_curvature_radpm, 0.50);
  EXPECT_DOUBLE_EQ(limits.maximum_steering_angle_rad, 0.5236);
  EXPECT_DOUBLE_EQ(limits.maximum_steering_rate_radps, 0.35);
  EXPECT_DOUBLE_EQ(limits.maximum_segment_length_m, 0.05);
}

TEST(LatticeGenerator, SparsePhysicallySharpArcStillRejects) {
  const auto generated = generateLatticeCandidates(
      sparseArcReference(1.5), egoOnArc(1.5, 5.0), LatticeConfig{}, 10.0);
  ASSERT_TRUE(generated.valid()) << generated.reason;
  const auto evaluated =
      evaluateCandidate(generated.candidates.front(), openCostmap());
  EXPECT_FALSE(evaluated.valid);
  EXPECT_EQ(evaluated.reject_reason, RejectReason::CURVATURE_LIMIT);
}

TEST(LatticeGenerator, AcceptsZeroSpeed) {
  auto ego = validEgo();
  ego.speed_mps = 0.0;
  EXPECT_TRUE(
      generateLatticeCandidates(straightReference(), ego, LatticeConfig{}, 10.0)
          .valid());
}

TEST(LatticeGenerator, RejectsFrameMismatch) {
  auto ego = validEgo();
  ego.frame_id = "odom";
  const auto result = generateLatticeCandidates(straightReference(), ego,
                                                LatticeConfig{}, 10.0);
  EXPECT_EQ(result.failure, AdmissionFailure::INVALID_FRAME);
  EXPECT_TRUE(result.candidates.empty());
}

TEST(LatticeGenerator, RejectsNonFiniteInput) {
  auto ego = validEgo();
  ego.x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
      generateLatticeCandidates(straightReference(), ego, LatticeConfig{}, 10.0)
          .failure,
      AdmissionFailure::NON_FINITE_INPUT);

  auto reference = straightReference();
  reference.points[2].yaw_rad = std::numeric_limits<double>::infinity();
  EXPECT_EQ(
      generateLatticeCandidates(reference, validEgo(), LatticeConfig{}, 10.0)
          .failure,
      AdmissionFailure::INVALID_REFERENCE);
}

TEST(LatticeGenerator, RejectsInvalidReferenceAndConfig) {
  auto reference = straightReference();
  reference.points.resize(1U);
  EXPECT_EQ(
      generateLatticeCandidates(reference, validEgo(), LatticeConfig{}, 10.0)
          .failure,
      AdmissionFailure::INVALID_REFERENCE);

  auto config = LatticeConfig{};
  config.sample_spacing_m = 0.0;
  EXPECT_EQ(
      generateLatticeCandidates(straightReference(), validEgo(), config, 10.0)
          .failure,
      AdmissionFailure::INVALID_CONFIG);

  config = LatticeConfig{};
  config.horizon_m = 1.0e12;
  config.sample_spacing_m = 1.0e-12;
  EXPECT_EQ(
      generateLatticeCandidates(straightReference(), validEgo(), config, 10.0)
          .failure,
      AdmissionFailure::INVALID_CONFIG);

  config = LatticeConfig{};
  config.sample_spacing_m = 0.051;
  EXPECT_EQ(
      generateLatticeCandidates(straightReference(), validEgo(), config, 10.0)
          .failure,
      AdmissionFailure::INVALID_CONFIG);

  config = LatticeConfig{};
  config.horizon_m = 50.0;
  config.sample_spacing_m = 1.0e-12;
  config.maximum_sample_count = std::numeric_limits<std::size_t>::max();
  EXPECT_EQ(
      generateLatticeCandidates(straightReference(), validEgo(), config, 10.0)
          .failure,
      AdmissionFailure::INVALID_CONFIG);
}

TEST(LatticeGenerator, RejectsStaleFutureAndReverseInput) {
  auto ego = validEgo();
  EXPECT_EQ(generateLatticeCandidates(straightReference(), ego, LatticeConfig{},
                                      10.21)
                .failure,
            AdmissionFailure::STALE_INPUT);

  ego.stamp_sec = 10.06;
  EXPECT_EQ(
      generateLatticeCandidates(straightReference(), ego, LatticeConfig{}, 10.0)
          .failure,
      AdmissionFailure::FUTURE_INPUT);

  ego = validEgo();
  ego.speed_mps = -0.1;
  EXPECT_EQ(
      generateLatticeCandidates(straightReference(), ego, LatticeConfig{}, 10.0)
          .failure,
      AdmissionFailure::REVERSE_SPEED);
}

TEST(LatticeGenerator, RejectsUnavailableHorizon) {
  auto reference = straightReference();
  reference.points.resize(10U);
  const auto result =
      generateLatticeCandidates(reference, validEgo(), LatticeConfig{}, 10.0);
  EXPECT_EQ(result.failure, AdmissionFailure::INVALID_REFERENCE);
  EXPECT_TRUE(result.candidates.empty());
}

TEST(LatticeGenerator, RejectsSnapshotMismatchAndInputSkew) {
  auto ego = validEgo();
  ego.snapshot_id = 2U;
  EXPECT_EQ(
      generateLatticeCandidates(straightReference(), ego, LatticeConfig{}, 10.0)
          .failure,
      AdmissionFailure::INVALID_TIMESTAMP);

  ego = validEgo();
  ego.stamp_sec = 10.06;
  auto config = LatticeConfig{};
  config.maximum_future_offset_sec = 0.10;
  EXPECT_EQ(
      generateLatticeCandidates(straightReference(), ego, config, 10.0).failure,
      AdmissionFailure::INVALID_TIMESTAMP);
}

} // namespace
} // namespace simple_state_lattice_planner
