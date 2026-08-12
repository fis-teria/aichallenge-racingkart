#include "state_lattice_overtake_planner/frenet_frame.hpp"
#include "state_lattice_overtake_planner/lattice_planner.hpp"
#include "state_lattice_overtake_planner/reference_override_contract.hpp"

#include "fixtures/dev3_20260727_225451.hpp"
#include "fixtures/dev3_20260728_010706.hpp"
#include "fixtures/dev3_20260728_120212.hpp"
#include "fixtures/dev3_20260728_135436.hpp"
#include "fixtures/dev3_20260728_184222.hpp"
#include "fixtures/dev3_20260728_213709.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace sl = state_lattice_overtake_planner;

namespace state_lattice_overtake_planner {
struct StateLatticeTestAccess {
  static std::optional<CandidateTrajectory> movingTargetFollowCandidate(
      const LatticePlanner &planner, const EgoState &ego,
      const OpponentState &target,
      const std::vector<OpponentState> &opponents) {
    return planner.movingTargetFollowCandidate(ego, target, opponents);
  }

  static bool outputHorizon(
      const LatticePlanner &planner, const EgoState &ego,
      const CandidateTrajectory &candidate,
      const std::vector<OpponentState> &opponents, double target_speed,
      std::vector<double> *d, std::vector<double> *speed,
      std::vector<double> *longitudinal_offsets_m,
      OutputHorizonDiagnostic *diagnostic) {
    return planner.outputHorizon(ego, candidate, opponents, target_speed, d,
                                 speed, longitudinal_offsets_m, diagnostic);
  }
};
} // namespace state_lattice_overtake_planner

namespace {
sl::FrenetFrame squareFrame() {
  sl::FrenetFrame frame;
  std::vector<sl::ReferencePoint> points(4);
  points[0].s = 0.0;
  points[0].x = 0.0;
  points[0].y = 0.0;
  points[0].yaw = 0.0;
  points[1].s = 10.0;
  points[1].x = 10.0;
  points[1].y = 0.0;
  points[1].yaw = M_PI_2;
  points[2].s = 20.0;
  points[2].x = 10.0;
  points[2].y = 10.0;
  points[2].yaw = M_PI;
  points[3].s = 30.0;
  points[3].x = 0.0;
  points[3].y = 10.0;
  points[3].yaw = -M_PI_2;
  EXPECT_TRUE(frame.setReference(points));
  return frame;
}

sl::FrenetFrame sampledSquareFrame() {
  std::vector<sl::ReferencePoint> points;
  double s = 0.0;
  const auto append = [&points, &s](double x, double y, double yaw) {
    sl::ReferencePoint point;
    point.s = s;
    point.x = x;
    point.y = y;
    point.yaw = yaw;
    point.kappa = 0.0;
    points.push_back(point);
    s += 1.0;
  };
  for (int x = 0; x < 30; ++x) {
    append(x, 0.0, 0.0);
  }
  for (int y = 0; y < 30; ++y) {
    append(30.0, y, M_PI_2);
  }
  for (int x = 30; x > 0; --x) {
    append(x, 30.0, M_PI);
  }
  for (int y = 30; y > 0; --y) {
    append(0.0, y, -M_PI_2);
  }
  sl::FrenetFrame frame;
  EXPECT_TRUE(frame.setReference(points));
  return frame;
}

sl::FrenetFrame straightFrame() {
  std::vector<sl::ReferencePoint> points;
  for (int x = 0; x <= 40; ++x) {
    sl::ReferencePoint point;
    point.s = static_cast<double>(x);
    point.x = static_cast<double>(x);
    point.y = 0.0;
    point.yaw = 0.0;
    point.kappa = 0.0;
    points.push_back(point);
  }
  sl::FrenetFrame frame;
  EXPECT_TRUE(frame.setReference(points));
  return frame;
}

sl::FrenetFrame shiftedStraightFrame(double lateral_shift_m) {
  std::vector<sl::ReferencePoint> points;
  constexpr double kSpacingM = 0.2;
  constexpr int kPointCount = 201;
  for (int index = 0; index < kPointCount; ++index) {
    const double x_m = static_cast<double>(index) * kSpacingM;
    sl::ReferencePoint point;
    point.s = x_m;
    point.x = x_m;
    point.y = lateral_shift_m;
    point.yaw = 0.0;
    point.kappa = 0.0;
    point.speed_mps = 8.0;
    points.push_back(point);
  }
  sl::FrenetFrame frame;
  EXPECT_TRUE(frame.setReference(points));
  return frame;
}

sl::GridMap testGrid(const sl::PlannerConfig &config, bool occupied = false) {
  const auto directory = std::filesystem::temp_directory_path() /
                         "state_lattice_overtake_planner_test_map";
  std::filesystem::create_directories(directory);
  const auto pgm = directory / "open.pgm";
  const auto yaml = directory / "open.yaml";
  {
    std::ofstream output(pgm, std::ios::binary);
    output << "P5\n600 600\n255\n";
    const std::vector<unsigned char> pixels(
        600U * 600U, occupied ? static_cast<unsigned char>(0U)
                              : static_cast<unsigned char>(255U));
    output.write(reinterpret_cast<const char *>(pixels.data()),
                 static_cast<std::streamsize>(pixels.size()));
  }
  {
    std::ofstream output(yaml);
    output << "image: open.pgm\nresolution: 0.1\norigin: [-10.0, -10.0, 0.0]\n"
           << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  }
  sl::GridMap map;
  std::string error;
  EXPECT_TRUE(map.load(yaml.string(), config, &error)) << error;
  return map;
}

sl::GridMap openGrid(const sl::PlannerConfig &config) {
  return testGrid(config);
}

sl::PlannerConfig sideRoleConfig(const std::string &own_vehicle_id) {
  sl::PlannerConfig config;
  config.own_vehicle_id = own_vehicle_id;
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  config.normal_mode_min_hold_sec = 0.0;
  return config;
}

sl::EgoState sideRoleEgo(sl::FrenetFrame &frame, double x_m, double d_m,
                         double speed_mps, double stamp_sec) {
  const auto pose = frame.frenetToCartesian(x_m, d_m);
  sl::EgoState ego;
  ego.x = pose.x;
  ego.y = pose.y;
  ego.yaw = pose.yaw;
  ego.curvature = pose.kappa;
  ego.speed_mps = speed_mps;
  ego.stamp_sec = stamp_sec;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  return ego;
}

sl::OpponentState sideRoleOpponent(sl::FrenetFrame &frame,
                                   const std::string &id, double x_m,
                                   double d_m, double speed_mps,
                                   double stamp_sec) {
  const auto pose = frame.frenetToCartesian(x_m, d_m);
  sl::OpponentState opponent;
  opponent.id = id;
  opponent.x = pose.x;
  opponent.y = pose.y;
  opponent.yaw = pose.yaw;
  opponent.speed_mps = speed_mps;
  opponent.vx_mps = speed_mps * std::cos(pose.yaw);
  opponent.vy_mps = speed_mps * std::sin(pose.yaw);
  opponent.stamp_sec = stamp_sec;
  opponent.uncertainty_x_m = 0.15;
  opponent.uncertainty_y_m = 0.15;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = opponent.frenet.valid;
  return opponent;
}

double unwrapNearForTest(double wrapped_s, double expected_s, double length_m) {
  if (length_m <= 0.0) {
    return wrapped_s;
  }
  return wrapped_s + std::round((expected_s - wrapped_s) / length_m) * length_m;
}

// Frozen copy of the pre-bounded projectContinuous search. This deliberately
// scans every physical segment in ascending index order so the optimized
// implementation is checked against the old distance and tie-break contract.
sl::FrenetPoint exhaustiveContinuousProjection(const sl::FrenetFrame &frame,
                                               double x, double y, double yaw,
                                               double expected_s,
                                               double half_width) {
  sl::FrenetPoint best;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  double best_expected_delta = std::numeric_limits<double>::infinity();
  if (frame.empty() || !std::isfinite(x) || !std::isfinite(y)) {
    return best;
  }
  const auto &points = frame.points();
  for (std::size_t i = 0U; i < points.size(); ++i) {
    const auto &a = points[i];
    const auto &b = points[(i + 1U) % points.size()];
    const double segment_s =
        i + 1U < points.size() ? b.s - a.s : frame.length() - a.s;
    const double unwrapped_a =
        unwrapNearForTest(a.s, expected_s, frame.length());
    const double unwrapped_b = unwrapped_a + segment_s;
    if (unwrapped_b < expected_s - half_width - 1.0e-9 ||
        unwrapped_a > expected_s + half_width + 1.0e-9) {
      continue;
    }
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double denom = vx * vx + vy * vy;
    if (denom <= 1.0e-12) {
      continue;
    }
    const double t =
        std::clamp(((x - a.x) * vx + (y - a.y) * vy) / denom, 0.0, 1.0);
    const double px = a.x + t * vx;
    const double py = a.y + t * vy;
    const double wrapped_s = frame.wrapS(a.s + t * segment_s);
    const double unwrapped_s =
        unwrapNearForTest(wrapped_s, expected_s, frame.length());
    const double expected_delta = std::abs(unwrapped_s - expected_s);
    if (expected_delta > half_width + 1.0e-9) {
      continue;
    }
    const double distance_sq = (x - px) * (x - px) + (y - py) * (y - py);
    if (distance_sq + 1.0e-10 < best_distance_sq ||
        (std::abs(distance_sq - best_distance_sq) <= 1.0e-10 &&
         expected_delta < best_expected_delta)) {
      const double segment_yaw = std::atan2(vy, vx);
      best.s = unwrapped_s;
      best.d =
          -std::sin(segment_yaw) * (x - px) + std::cos(segment_yaw) * (y - py);
      best.yaw_error = sl::normalizeAngle(yaw - segment_yaw);
      best.segment_index = i;
      best.valid = true;
      best_distance_sq = distance_sq;
      best_expected_delta = expected_delta;
    }
  }
  return best;
}
} // namespace

TEST(FrenetFrame, ProjectsContinuouslyAndUnwrapsLoopBoundary) {
  const auto frame = squareFrame();
  const auto straight = frame.project(4.0, 1.0, 0.0);
  ASSERT_TRUE(straight.valid);
  EXPECT_NEAR(straight.s, 4.0, 1.0e-9);
  EXPECT_NEAR(straight.d, 1.0, 1.0e-9);
  const auto near_lap = frame.projectContinuous(0.0, 0.5, -M_PI_2, 39.5, 2.0);
  ASSERT_TRUE(near_lap.valid);
  EXPECT_GT(near_lap.s, 39.0);
  EXPECT_NEAR(frame.wrapS(near_lap.s), 39.5, 0.6);
  const auto unique_near_lap =
      frame.projectContinuousUnique(0.0, 0.5, -M_PI_2, 39.5, 2.0);
  ASSERT_TRUE(unique_near_lap.valid);
  EXPECT_NEAR(unique_near_lap.s, near_lap.s, 1.0e-9);
}

TEST(FrenetFrame, UniqueContinuousProjectionRejectsDistinctEqualCandidates) {
  sl::FrenetFrame frame;
  std::string error;
  const double diagonal = std::sqrt(8.0);
  ASSERT_TRUE(frame.setReference(
      {{0.0, 0.0, M_PI_4, 0.0, 0.0, 1.0},
       {2.0, 2.0, M_PI, diagonal, 0.0, 1.0},
       {0.0, 2.0, -M_PI_4, diagonal + 2.0, 0.0, 1.0},
       {2.0, 0.0, M_PI, 2.0 * diagonal + 2.0, 0.0, 1.0}},
      &error)) << error;

  const double expected_s = diagonal + 1.0;
  const auto legacy =
      frame.projectContinuous(1.0, 1.0, 0.0, expected_s, 5.0);
  ASSERT_TRUE(legacy.valid);
  const auto unique =
      frame.projectContinuousUnique(1.0, 1.0, 0.0, expected_s, 5.0);
  EXPECT_FALSE(unique.valid);
}

TEST(FrenetFrame, UniqueContinuousProjectionKeepsInclusiveTwoMetreWindow) {
  const auto frame = squareFrame();
  const auto boundary =
      frame.projectContinuousUnique(4.0, 1.0, 0.0, 2.0, 2.0);
  ASSERT_TRUE(boundary.valid);
  EXPECT_NEAR(boundary.s, 4.0, 1.0e-9);
  const auto outside =
      frame.projectContinuousUnique(4.01, 1.0, 0.0, 2.0, 2.0);
  EXPECT_FALSE(outside.valid);
}

TEST(FrenetFrame, BoundedContinuousProjectionMatchesLegacyExhaustiveSearch) {
  const auto frame = sampledSquareFrame();
  const std::array<double, 9> expected_values{-121.0, -1.0, 0.0,   1.0,  29.5,
                                              59.5,   89.5, 119.5, 241.0};
  const std::array<double, 5> half_widths{0.0, 0.25, 1.0, 4.0, 12.0};
  const std::array<double, 5> lateral_offsets{-2.0, -0.25, 0.0, 0.25, 2.0};

  for (const double expected_s : expected_values) {
    const auto reference = frame.interpolate(expected_s);
    for (const double half_width : half_widths) {
      for (const double lateral_offset : lateral_offsets) {
        const double x = reference.x - std::sin(reference.yaw) * lateral_offset;
        const double y = reference.y + std::cos(reference.yaw) * lateral_offset;
        const auto expected = exhaustiveContinuousProjection(
            frame, x, y, reference.yaw, expected_s, half_width);
        const auto actual = frame.projectContinuous(x, y, reference.yaw,
                                                    expected_s, half_width);
        ASSERT_EQ(actual.valid, expected.valid)
            << "expected_s=" << expected_s << " half_width=" << half_width
            << " lateral_offset=" << lateral_offset;
        if (!expected.valid) {
          continue;
        }
        EXPECT_EQ(actual.segment_index, expected.segment_index);
        EXPECT_DOUBLE_EQ(actual.s, expected.s);
        EXPECT_DOUBLE_EQ(actual.d, expected.d);
        EXPECT_DOUBLE_EQ(actual.yaw_error, expected.yaw_error);
      }
    }
  }
}

TEST(FrenetFrame,
     BoundedContinuousProjectionMatchesExhaustiveAcrossAllCourseSegments) {
  sl::FrenetFrame frame;
  std::string error;
  const std::string share = TEST_MPC_SOURCE_DIR;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  // The current final_ver3 artifact contains 350 reference points, hence 350
  // physical segments including the explicit last-to-first closure.
  ASSERT_EQ(frame.points().size(), 350U);

  const std::array<double, 3> lap_offsets{-frame.length(), 0.0, frame.length()};
  const std::array<double, 3> lateral_offsets{-1.0, 0.0, 1.0};
  for (const auto &point : frame.points()) {
    for (const double lap_offset : lap_offsets) {
      const double expected_s = point.s + lap_offset;
      for (const double lateral_offset : lateral_offsets) {
        const double x = point.x - std::sin(point.yaw) * lateral_offset;
        const double y = point.y + std::cos(point.yaw) * lateral_offset;
        const auto expected = exhaustiveContinuousProjection(
            frame, x, y, point.yaw, expected_s, 2.0);
        const auto actual =
            frame.projectContinuous(x, y, point.yaw, expected_s, 2.0);
        const auto unique =
            frame.projectContinuousUnique(x, y, point.yaw, expected_s, 2.0);
        ASSERT_EQ(actual.valid, expected.valid)
            << "reference_s=" << point.s << " lap_offset=" << lap_offset
            << " lateral_offset=" << lateral_offset;
        ASSERT_TRUE(expected.valid);
        ASSERT_TRUE(unique.valid)
            << "reference_s=" << point.s << " lap_offset=" << lap_offset
            << " lateral_offset=" << lateral_offset;
        EXPECT_EQ(actual.segment_index, expected.segment_index);
        EXPECT_DOUBLE_EQ(actual.s, expected.s);
        EXPECT_DOUBLE_EQ(actual.d, expected.d);
        EXPECT_DOUBLE_EQ(actual.yaw_error, expected.yaw_error);
        EXPECT_EQ(unique.segment_index, actual.segment_index);
        EXPECT_DOUBLE_EQ(unique.s, actual.s);
        EXPECT_DOUBLE_EQ(unique.d, actual.d);
        EXPECT_DOUBLE_EQ(unique.yaw_error, actual.yaw_error);
      }
    }
  }
}

TEST(FrenetFrame, CourseCenterlineIsClosedAndRoundTripsAcrossTheFullLap) {
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error))
      << error;
  ASSERT_EQ(frame.points().size(), 350U);
  EXPECT_GT(frame.length(), frame.points().back().s);
  EXPECT_LT(frame.length() - frame.points().back().s, 1.1);

  for (const auto &point : frame.points()) {
    for (const double d_m : {-1.0, 0.0, 1.0}) {
      const auto cartesian = frame.frenetToCartesian(point.s, d_m);
      const auto projected = frame.projectContinuous(
          cartesian.x, cartesian.y, cartesian.yaw, point.s, 2.0);
      ASSERT_TRUE(projected.valid) << "s=" << point.s << " d=" << d_m;
      // At d=0 this is the exact source polyline.  Offset polylines around a
      // corner are not arc-length preserving, so their nearest projection may
      // move slightly along the adjacent segment; keep that geometric error
      // explicitly bounded instead of pretending the station is identical.
      if (std::abs(d_m) < 1.0e-9) {
        EXPECT_NEAR(projected.s, point.s, 1.0e-5);
        EXPECT_NEAR(projected.d, d_m, 1.0e-5);
      } else {
        EXPECT_NEAR(projected.s, point.s, 0.6);
        EXPECT_GT(projected.d * d_m, 0.0);
        EXPECT_GT(std::abs(projected.d), 0.8);
      }
    }
  }
}

TEST(FrenetFrame, Runtime21CenterlineCoordinatesPutEgoAndD2LeftOfCenter) {
  sl::FrenetFrame centerline;
  sl::FrenetFrame controller_reference;
  std::string error;
  ASSERT_TRUE(centerline.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error))
      << error;
  ASSERT_TRUE(controller_reference.loadCsv(
      std::string(TEST_MPC_SOURCE_DIR) +
          "/env/final_ver3/traj_mincurv_manual.csv",
      &error))
      << error;

  const auto ego = centerline.project(89633.252, 43124.957, 2.264);
  ASSERT_TRUE(ego.valid);
  EXPECT_NEAR(ego.d, 1.083, 0.01);

  const auto old_d2 = controller_reference.frenetToCartesian(32.231899, 1.978059);
  const auto d2 = centerline.project(old_d2.x, old_d2.y, old_d2.yaw);
  ASSERT_TRUE(d2.valid);
  EXPECT_NEAR(d2.d, 1.512, 0.01);
  EXPECT_GT(ego.d, 0.0);
  EXPECT_GT(d2.d, 0.0);
  EXPECT_LT(0.0, d2.d);
  EXPECT_LT(-1.0, d2.d);
  EXPECT_GT(centerline.forwardDeltaS(ego.s, d2.s), 5.0);
}

TEST(FrenetFrame, RejectsInvalidCurvatureAndSpeedColumns) {
  auto frame = sampledSquareFrame();
  auto points = frame.points();
  points.front().kappa = std::numeric_limits<double>::quiet_NaN();
  sl::FrenetFrame invalid;
  std::string error;
  EXPECT_FALSE(invalid.setReference(points, &error));
  points = frame.points();
  points.front().speed_mps = -1.0;
  EXPECT_FALSE(invalid.setReference(points, &error));
}

TEST(Quintic, SatisfiesPositionYawAndCurvatureBoundaries) {
  sl::ParametricQuintic polynomial;
  ASSERT_TRUE(
      polynomial.configure({0.0, 0.0, 0.0}, 0.0, {8.0, 1.0, 0.1}, 0.02, 1.0));
  const auto start = polynomial.sample(0.0);
  const auto goal = polynomial.sample(1.0);
  EXPECT_NEAR(start.x, 0.0, 1.0e-10);
  EXPECT_NEAR(start.y, 0.0, 1.0e-10);
  EXPECT_NEAR(start.yaw, 0.0, 1.0e-10);
  EXPECT_NEAR(start.kappa, 0.0, 1.0e-9);
  EXPECT_NEAR(goal.x, 8.0, 1.0e-9);
  EXPECT_NEAR(goal.y, 1.0, 1.0e-9);
  EXPECT_NEAR(goal.yaw, 0.1, 1.0e-9);
  EXPECT_NEAR(goal.kappa, 0.02, 1.0e-8);
}

TEST(LatticePlanner, RejectsSelfIntersectingDenseTrajectory) {
  const sl::PlannerConfig config;
  sl::LatticePlanner planner(config, nullptr, nullptr);
  sl::CandidateTrajectory candidate;
  candidate.representative.resize(5U);
  candidate.dense.resize(4U);
  candidate.dense[0].x = 0.0;
  candidate.dense[0].y = 0.0;
  candidate.dense[1].x = 2.0;
  candidate.dense[1].y = 2.0;
  candidate.dense[2].x = 0.0;
  candidate.dense[2].y = 2.0;
  candidate.dense[3].x = 2.0;
  candidate.dense[3].y = 0.0;
  EXPECT_FALSE(planner.evaluateTrajectory(&candidate, {}));
  EXPECT_EQ(candidate.rejection_reason, "self_intersection");
}

TEST(FrontDetector, InitialAndPeriodicSweepsHaveOneEnterFiveRelease) {
  const sl::PlannerConfig config;
  auto frame = squareFrame();
  sl::FrontDetector detector(config);
  sl::EgoState ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 0.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.x = 1.0;
  opponent.y = 0.0;
  opponent.yaw = 0.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;
  ASSERT_EQ(detector.update(ego, {opponent}, frame),
            std::optional<std::string>("d2"));
  EXPECT_TRUE(detector.detected());
  ASSERT_EQ(detector.sweepStarts().size(), detector.currentTerminals().size());
  for (const auto &start : detector.sweepStarts()) {
    EXPECT_DOUBLE_EQ(start.x, ego.x);
    EXPECT_DOUBLE_EQ(start.y, ego.y);
  }
  const auto first_terminals = detector.currentTerminals();
  detector.update(ego, {}, frame);
  ASSERT_EQ(detector.sweepStarts().size(), first_terminals.size());
  for (const auto &start : detector.sweepStarts()) {
    EXPECT_DOUBLE_EQ(start.x, ego.x);
    EXPECT_DOUBLE_EQ(start.y, ego.y);
  }
  for (int i = 0; i < 3; ++i) {
    detector.update(ego, {}, frame);
    EXPECT_TRUE(detector.detected());
  }
  detector.update(ego, {}, frame);
  EXPECT_FALSE(detector.detected());
}

TEST(FrontDetector, DetectsOpponentEnteringInsideDerivedTransitionEnvelope) {
  sl::PlannerConfig config;
  config.front_release_cycles = 1;
  auto frame = sampledSquareFrame();
  sl::FrontDetector detector(config);
  sl::EgoState ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 8.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  EXPECT_FALSE(detector.update(ego, {}, frame).has_value());

  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.x = 2.5;
  opponent.y = 0.0;
  opponent.yaw = 0.0;
  opponent.uncertainty_x_m = 0.15;
  opponent.uncertainty_y_m = 0.15;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;
  EXPECT_EQ(detector.update(ego, {opponent}, frame),
            std::optional<std::string>("d2"));
  EXPECT_GE(detector.detectionRadiusM(),
            sl::derivedFrontDetectionRadius(config, opponent));
  double expected_transition_distance_m = 0.0;
  for (const double goal_d_m : config.lateral_targets_m) {
    expected_transition_distance_m =
        std::max(expected_transition_distance_m,
                 sl::requiredLateralTransitionDistance(config, ego.speed_mps,
                                                       ego.frenet.d, goal_d_m));
  }
  EXPECT_NEAR(detector.maximumTransitionDistanceM(),
              expected_transition_distance_m, 1.0e-9);
}

TEST(FrontDetector, ForwardTargetWinsOverParallelFallbackAndRearIsExcluded) {
  const sl::PlannerConfig config;
  auto frame = sampledSquareFrame();
  sl::EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 3.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;

  sl::OpponentState forward;
  forward.id = "forward";
  forward.x = 8.0;
  forward.y = 0.0;
  forward.yaw = 0.0;
  forward.frenet = frame.project(forward.x, forward.y, forward.yaw);
  forward.valid = forward.frenet.valid;

  sl::OpponentState parallel;
  parallel.id = "parallel";
  parallel.x = 5.0;
  parallel.y = 2.5;
  parallel.yaw = 0.0;
  parallel.frenet = frame.project(parallel.x, parallel.y, parallel.yaw);
  parallel.valid = parallel.frenet.valid;

  sl::OpponentState rear = forward;
  rear.id = "rear";
  rear.x = 2.0;
  rear.frenet = frame.project(rear.x, rear.y, rear.yaw);
  rear.valid = rear.frenet.valid;

  sl::FrontDetector detector(config);
  EXPECT_EQ(detector.update(ego, {parallel, rear, forward}, frame),
            std::optional<std::string>("forward"));
  detector.reset();
  EXPECT_EQ(detector.update(ego, {parallel, rear}, frame),
            std::optional<std::string>("parallel"));
  detector.reset();
  EXPECT_FALSE(detector.update(ego, {rear}, frame).has_value());
}

TEST(LatticePlanner, ParallelClearanceRanksOnlyInsidePassAndStopCostClasses) {
  sl::PlannerConfig config;
  config.front_enter_cycles = 1;
  config.normal_mode_min_hold_sec = 0.0;
  config.stop_cost = 100000;
  // This test isolates the legacy pass-cost ranking below. Preventive
  // side-by-side role behavior has dedicated tests later in this file.
  config.preventive_side_role_trigger_clearance_m =
      config.opponent_hard_clearance_m + 1.0e-3;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  sl::EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 2.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  sl::OpponentState forward;
  forward.id = "forward";
  forward.x = 15.0;
  forward.y = 0.0;
  forward.yaw = 0.0;
  forward.speed_mps = 1.0;
  forward.vx_mps = 1.0;
  forward.stamp_sec = 10.0;
  forward.frenet = frame.project(forward.x, forward.y, forward.yaw);
  forward.valid = forward.frenet.valid;

  sl::OpponentState parallel = forward;
  parallel.id = "parallel";
  parallel.x = 5.0;
  parallel.y = 2.0;
  parallel.speed_mps = 2.0;
  parallel.vx_mps = 2.0;
  parallel.frenet = frame.project(parallel.x, parallel.y, parallel.yaw);
  parallel.valid = parallel.frenet.valid;

  sl::LatticePlanner planner(config, &frame, &map);
  const auto output = planner.update(ego, {forward, parallel}, true, 10.0);
  ASSERT_TRUE(planner.selected().has_value()) << output.reason;
  EXPECT_EQ(output.mode, sl::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_LT(planner.selected()->goal_d_m, 0.0);
  EXPECT_LT(planner.selected()->total_cost, config.stop_cost);
  EXPECT_GT(planner.selected()->opponent_clearance_recovery_m, 0.0);

  const auto stale = planner.update(ego, {forward, parallel}, false, 10.1);
  EXPECT_EQ(stale.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_TRUE(stale.emergency_stop);
  EXPECT_EQ(stale.reason, "invalid_or_stale_input");
  EXPECT_EQ(stale.generated_candidates, 0);
  EXPECT_TRUE(stale.lateral_offsets_m.empty());
}

TEST(LatticePlanner, DistinctOutputFrameEncodesAndValidatesControllerOffsets) {
  sl::PlannerConfig config;
  config.front_enter_cycles = 1;
  config.normal_mode_min_hold_sec = 0.0;
  config.stop_cost = 100000;
  config.preventive_side_role_trigger_clearance_m =
      config.opponent_hard_clearance_m + 1.0e-3;
  auto planning_frame = shiftedStraightFrame(1.0);
  auto output_frame = shiftedStraightFrame(0.0);
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  auto ego = sideRoleEgo(planning_frame, 5.0, 0.0, 2.0, 10.0);
  auto forward =
      sideRoleOpponent(planning_frame, "forward", 15.0, 0.0, 1.0, 10.0);
  auto parallel =
      sideRoleOpponent(planning_frame, "parallel", 5.0, 2.0, 2.0, 10.0);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto output = planner.update(ego, {forward, parallel}, true, 10.0);
  ASSERT_TRUE(planner.selected().has_value())
      << output.reason << " / "
      << sl::toString(output.output_horizon_diagnostic.failure) << " waypoint="
      << output.output_horizon_diagnostic.waypoint_index << " observed="
      << output.output_horizon_diagnostic.observed_value;
  ASSERT_TRUE(output.safe_lateral) << output.reason;
  ASSERT_EQ(output.lateral_offsets_m.size(),
            static_cast<std::size_t>(config.horizon_points));
  ASSERT_EQ(output.lateral_offsets_m.size(),
            output.longitudinal_offsets_m.size());
  EXPECT_NEAR(output.lateral_offsets_m.front(), 1.0, 1.0e-6);
  EXPECT_EQ(output.output_horizon_diagnostic.failure,
            sl::OutputHorizonFailure::NONE);
  EXPECT_TRUE(std::all_of(output.lateral_offsets_m.begin(),
                          output.lateral_offsets_m.end(),
                          [](double value) { return std::isfinite(value); }));
}

TEST(LatticePlanner, TwoFrameContinuousProjectionRejectsBackwardMapping) {
  sl::PlannerConfig config;
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  auto planning_frame = shiftedStraightFrame(1.0);
  auto output_frame = shiftedStraightFrame(0.0);
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  auto ego = sideRoleEgo(planning_frame, 5.0, 0.0, 1.0, 10.0);
  sl::CandidateTrajectory candidate;
  for (int index = 0; index < 4; ++index) {
    sl::TrajectoryPoint point;
    point.x = ego.x - static_cast<double>(index);
    point.y = ego.y;
    point.yaw = M_PI;
    point.speed_mps = 1.0;
    point.kappa = 0.0;
    candidate.dense.push_back(point);
  }

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  std::vector<double> d;
  std::vector<double> speed;
  std::vector<double> longitudinal_offsets_m;
  sl::OutputHorizonDiagnostic diagnostic;
  EXPECT_FALSE(sl::StateLatticeTestAccess::outputHorizon(
      planner, ego, candidate, {}, 1.0, &d, &speed,
      &longitudinal_offsets_m, &diagnostic));
  EXPECT_EQ(diagnostic.failure, sl::OutputHorizonFailure::INVALID_INPUT);
  EXPECT_EQ(diagnostic.waypoint_index, 1);
  EXPECT_LT(diagnostic.observed_value, -config.projection_max_backward_m);
  EXPECT_DOUBLE_EQ(diagnostic.limit_value,
                   config.projection_max_backward_m);
}

TEST(LatticePlanner, TwoFrameContinuousProjectionExtendsTerminalOffsetForward) {
  sl::PlannerConfig config;
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  auto planning_frame = shiftedStraightFrame(1.0);
  auto output_frame = shiftedStraightFrame(0.0);
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  auto ego = sideRoleEgo(planning_frame, 5.0, 0.0, 1.0, 10.0);
  sl::CandidateTrajectory candidate;
  for (int index = 0; index < 2; ++index) {
    sl::TrajectoryPoint point;
    point.x = ego.x + 0.2 * static_cast<double>(index);
    point.y = ego.y;
    point.yaw = 0.0;
    point.speed_mps = 1.0;
    point.kappa = 0.0;
    candidate.dense.push_back(point);
  }

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  std::vector<double> d;
  std::vector<double> speed;
  std::vector<double> longitudinal_offsets_m;
  sl::OutputHorizonDiagnostic diagnostic;
  EXPECT_TRUE(sl::StateLatticeTestAccess::outputHorizon(
      planner, ego, candidate, {}, 1.0, &d, &speed,
      &longitudinal_offsets_m, &diagnostic));
  EXPECT_EQ(diagnostic.failure, sl::OutputHorizonFailure::NONE);
  ASSERT_EQ(d.size(), static_cast<std::size_t>(config.horizon_points));
  ASSERT_EQ(d.size(), longitudinal_offsets_m.size());
  EXPECT_TRUE(std::all_of(d.begin(), d.end(), [](double value) {
    return std::abs(value - 1.0) <= 1.0e-6;
  }));
  for (std::size_t index = 1U; index < longitudinal_offsets_m.size(); ++index) {
    EXPECT_GT(longitudinal_offsets_m[index],
              longitudinal_offsets_m[index - 1U]);
  }
}

TEST(FrontDetector, Dev3D1StartPrefersTransitionCorridorOverFrenetGap) {
  const sl::PlannerConfig config;
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;

  const auto &ego_source = sl::test_fixture::kD1StartTargetEgo;
  sl::EgoState ego;
  ego.x = ego_source.x_m;
  ego.y = ego_source.y_m;
  ego.yaw = ego_source.yaw_rad;
  ego.speed_mps = ego_source.speed_mps;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  std::vector<sl::OpponentState> opponents;
  for (const auto &source : sl::test_fixture::kD1StartTargetOpponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.frenet = frame.project(opponent.x, opponent.y, 0.0);
    ASSERT_TRUE(opponent.frenet.valid);
    opponent.yaw = frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.uncertainty_x_m = 0.15;
    opponent.uncertainty_y_m = 0.15;
    opponent.valid = opponent.frenet.valid;
    ASSERT_TRUE(opponent.valid);
    opponents.push_back(opponent);
  }

  sl::FrontDetector detector(config);
  EXPECT_EQ(detector.update(ego, opponents, frame),
            std::optional<std::string>("d3"));
  const auto &diagnostic = detector.diagnostic();
  ASSERT_EQ(diagnostic.opponent_count, 2U);
  EXPECT_STREQ(diagnostic.opponents[0].opponent_id.data(), "d2");
  EXPECT_EQ(diagnostic.opponents[0].target_class,
            sl::FrontTargetClass::FORWARD);
  EXPECT_STREQ(diagnostic.opponents[1].opponent_id.data(), "d3");
  EXPECT_EQ(diagnostic.opponents[1].target_class,
            sl::FrontTargetClass::FORWARD);
  EXPECT_LT(diagnostic.opponents[0].forward_gap_m,
            diagnostic.opponents[1].forward_gap_m);
  EXPECT_GT(diagnostic.opponents[0].minimum_sweep_distance_m,
            diagnostic.opponents[1].minimum_sweep_distance_m);

  // A non-finite sweep must not enter the new comparator. The complete
  // FORWARD set falls back to the former deterministic Frenet-gap order,
  // rather than treating the unknown sweep as a distant opponent.
  auto invalid_sweep_opponents = opponents;
  invalid_sweep_opponents[1].x = std::numeric_limits<double>::quiet_NaN();
  detector.reset();
  EXPECT_EQ(detector.update(ego, invalid_sweep_opponents, frame),
            std::optional<std::string>("d2"));
}

TEST(EarlyAwareSelector, StationaryTargetActivatesAtInclusiveDynamicBoundary) {
  sl::PlannerConfig config;
  config.early_aware_required_fresh_stamps = 2;
  auto frame = straightFrame();
  sl::EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 6.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  sl::OpponentState stationary;
  stationary.id = "stationary";
  stationary.x = 30.5; // 4 + 6 * 0.25 + 6^2 / (2 * 0.9) from ego at x=5.
  stationary.y = 0.0;
  stationary.yaw = 0.0;
  stationary.vx_mps = 0.0;
  stationary.vy_mps = 0.0;
  stationary.stamp_sec = 10.0;
  stationary.frenet = frame.project(stationary.x, stationary.y, stationary.yaw);
  stationary.valid = stationary.frenet.valid;

  sl::EarlyAwareSelector selector(config);
  selector.update(ego, {stationary}, frame, true, 10.0);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::CANDIDATE);
  EXPECT_DOUBLE_EQ(selector.diagnostic().dynamic_distance_m, 25.5);
  EXPECT_DOUBLE_EQ(selector.diagnostic().forward_delta_s_m, 25.5);
  ego.stamp_sec = 10.1;
  stationary.stamp_sec = 10.1;
  selector.update(ego, {stationary}, frame, true, 10.1);
  EXPECT_TRUE(selector.diagnostic().active);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::ACTIVE);
  EXPECT_EQ(selector.diagnostic().distinct_fresh_stamp_count, 2);
  EXPECT_STREQ(selector.diagnostic().target_id.data(), "stationary");
}

TEST(EarlyAwareSelector, EqualRecedingInvalidAndReverseObservationsClear) {
  sl::PlannerConfig config;
  auto frame = straightFrame();
  sl::EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 4.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  sl::OpponentState opponent;
  opponent.id = "target";
  opponent.x = 12.0;
  opponent.y = 0.0;
  opponent.yaw = 0.0;
  opponent.vx_mps = 4.0;
  opponent.vy_mps = 0.0;
  opponent.stamp_sec = 10.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = opponent.frenet.valid;
  sl::EarlyAwareSelector selector(config);

  selector.update(ego, {opponent}, frame, true, 10.0);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::INACTIVE);
  opponent.vx_mps = 5.0;
  selector.update(ego, {opponent}, frame, true, 10.0);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::INACTIVE);
  opponent.vx_mps = 0.0;
  opponent.yaw = M_PI;
  selector.update(ego, {opponent}, frame, true, 10.0);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::INACTIVE);
  opponent.yaw = 0.0;
  opponent.vx_mps = std::numeric_limits<double>::quiet_NaN();
  selector.update(ego, {opponent}, frame, true, 10.0);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::INACTIVE);
  opponent.vx_mps = 0.0;
  opponent.stamp_sec = 11.0;
  selector.update(ego, {opponent}, frame, true, 10.0);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::INACTIVE);
  opponent.stamp_sec = 9.0;
  selector.update(ego, {opponent}, frame, true, 10.0);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::INACTIVE);
}

TEST(EarlyAwareSelector, ClampsDynamicDistanceAtEightAndThirtyMeters) {
  sl::PlannerConfig config;
  config.early_aware_required_fresh_stamps = 1;
  auto frame = straightFrame();
  sl::EgoState ego;
  ego.x = 1.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 1.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  sl::OpponentState target;
  target.id = "target";
  target.x = 9.0;
  target.y = 0.0;
  target.yaw = 0.0;
  target.vx_mps = 0.0;
  target.vy_mps = 0.0;
  target.stamp_sec = 10.0;
  target.frenet = frame.project(target.x, target.y, target.yaw);
  target.valid = target.frenet.valid;
  sl::EarlyAwareSelector selector(config);
  selector.update(ego, {target}, frame, true, 10.0);
  EXPECT_TRUE(selector.diagnostic().active);
  EXPECT_DOUBLE_EQ(selector.diagnostic().dynamic_distance_m, 8.0);

  ego.x = 1.0;
  ego.speed_mps = 20.0;
  ego.stamp_sec = 11.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  target.x = 31.0;
  target.stamp_sec = 11.0;
  target.frenet = frame.project(target.x, target.y, target.yaw);
  selector.update(ego, {target}, frame, true, 11.0);
  EXPECT_TRUE(selector.diagnostic().active);
  EXPECT_DOUBLE_EQ(selector.diagnostic().dynamic_distance_m, 30.0);
  EXPECT_DOUBLE_EQ(selector.diagnostic().forward_delta_s_m, 30.0);
}

TEST(EarlyAwareSelector, WrapsFrenetForwardDeltaAcrossTrackBoundary) {
  sl::PlannerConfig config;
  config.early_aware_required_fresh_stamps = 1;
  auto frame = sampledSquareFrame();
  const auto ego_pose = frame.frenetToCartesian(119.0, 0.0);
  const auto target_pose = frame.frenetToCartesian(2.0, 0.0);
  sl::EgoState ego;
  ego.x = ego_pose.x;
  ego.y = ego_pose.y;
  ego.yaw = ego_pose.yaw;
  ego.speed_mps = 4.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  sl::OpponentState target;
  target.id = "wrapped";
  target.x = target_pose.x;
  target.y = target_pose.y;
  target.yaw = target_pose.yaw;
  target.vx_mps = 0.0;
  target.vy_mps = 0.0;
  target.stamp_sec = 10.0;
  target.frenet = frame.project(target.x, target.y, target.yaw);
  target.valid = target.frenet.valid;
  sl::EarlyAwareSelector selector(config);
  selector.update(ego, {target}, frame, true, 10.0);
  EXPECT_TRUE(selector.diagnostic().active);
  EXPECT_LT(selector.diagnostic().forward_delta_s_m, 8.0);
  EXPECT_STREQ(selector.diagnostic().target_id.data(), "wrapped");
}

TEST(EarlyAwareSelector, UsesLocalCurvedTrackTangentAndDeterministicTieBreak) {
  sl::PlannerConfig config;
  config.early_aware_required_fresh_stamps = 1;
  auto frame = sampledSquareFrame();
  const auto ego_pose = frame.frenetToCartesian(32.0, 0.0);
  const auto target_pose = frame.frenetToCartesian(36.0, 0.0);
  sl::EgoState ego;
  ego.x = ego_pose.x;
  ego.y = ego_pose.y;
  ego.yaw = ego_pose.yaw;
  ego.speed_mps = 4.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  sl::OpponentState rejected;
  rejected.id = "rejected";
  rejected.x = target_pose.x;
  rejected.y = target_pose.y;
  rejected.yaw = 0.0;
  rejected.vx_mps = 0.0;
  rejected.vy_mps = 0.0;
  rejected.stamp_sec = 10.0;
  rejected.frenet = frame.project(rejected.x, rejected.y, rejected.yaw);
  rejected.valid = rejected.frenet.valid;
  sl::EarlyAwareSelector selector(config);
  selector.update(ego, {rejected}, frame, true, 10.0);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::INACTIVE);

  auto b = rejected;
  b.id = "b";
  b.yaw = target_pose.yaw;
  b.frenet = frame.project(b.x, b.y, b.yaw);
  b.valid = b.frenet.valid;
  auto a = b;
  a.id = "a";
  selector.update(ego, {b, a}, frame, true, 10.0);
  EXPECT_TRUE(selector.diagnostic().active);
  EXPECT_STREQ(selector.diagnostic().target_id.data(), "a");
}

TEST(EarlyAwareSelector,
     MultipleOpponentsDoNotChangeFrontDetectorOrPlannerRole) {
  sl::PlannerConfig config;
  config.front_enter_cycles = 1;
  config.preventive_side_role_trigger_clearance_m =
      config.opponent_hard_clearance_m + 1.0e-3;
  auto frame = straightFrame();
  sl::EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 6.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  sl::OpponentState nearer;
  nearer.id = "nearer";
  nearer.x = 12.0;
  nearer.y = 0.0;
  nearer.yaw = 0.0;
  nearer.vx_mps = 0.0;
  nearer.vy_mps = 0.0;
  nearer.stamp_sec = 10.0;
  nearer.frenet = frame.project(nearer.x, nearer.y, nearer.yaw);
  nearer.valid = nearer.frenet.valid;
  auto farther = nearer;
  farther.id = "farther";
  farther.x = 20.0;
  farther.frenet = frame.project(farther.x, farther.y, farther.yaw);
  farther.valid = farther.frenet.valid;

  sl::FrontDetector detector(config);
  const auto before = detector.update(ego, {farther, nearer}, frame);
  sl::EarlyAwareSelector selector(config);
  selector.update(ego, {farther, nearer}, frame, true, 10.0);
  const auto after = detector.update(ego, {farther, nearer}, frame);
  EXPECT_EQ(before, after);
  EXPECT_EQ(selector.diagnostic().state, sl::EarlyAwareState::CANDIDATE);
  EXPECT_STREQ(selector.diagnostic().target_id.data(), "nearer");
  // Selector is not a pass/role latch and has no authority-bearing API.
  EXPECT_FALSE(selector.diagnostic().active);
}

TEST(LatticePlanner, EarlyAwareDoesNotChangePassRoleOrOutputAuthority) {
  sl::PlannerConfig observed_config;
  observed_config.normal_mode_min_hold_sec = 0.0;
  observed_config.preventive_side_role_trigger_clearance_m =
      observed_config.opponent_hard_clearance_m + 1.0e-3;
  auto disabled_config = observed_config;
  disabled_config.early_aware_enabled = false;
  auto frame = straightFrame();
  auto map = openGrid(observed_config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, observed_config));
  sl::EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 6.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  sl::OpponentState target;
  target.id = "early_only";
  target.x = 20.0;
  target.y = 0.0;
  target.yaw = 0.0;
  target.vx_mps = 0.0;
  target.vy_mps = 0.0;
  target.stamp_sec = 10.0;
  target.frenet = frame.project(target.x, target.y, target.yaw);
  target.valid = target.frenet.valid;
  sl::LatticePlanner observed(observed_config, &frame, &map);
  sl::LatticePlanner disabled(disabled_config, &frame, &map);
  const auto first_observed = observed.update(ego, {target}, true, 10.0);
  const auto first_disabled = disabled.update(ego, {target}, true, 10.0);
  ego.stamp_sec = 10.1;
  target.stamp_sec = 10.1;
  const auto output = observed.update(ego, {target}, true, 10.1);
  const auto baseline = disabled.update(ego, {target}, true, 10.1);

  EXPECT_EQ(first_observed.mode, first_disabled.mode);
  EXPECT_EQ(output.mode, baseline.mode);
  EXPECT_EQ(output.active, baseline.active);
  EXPECT_EQ(output.target_id, baseline.target_id);
  EXPECT_EQ(output.speed_cap_mps, baseline.speed_cap_mps);
  EXPECT_EQ(output.candidate_speed_limit_mps,
            baseline.candidate_speed_limit_mps);
  EXPECT_EQ(output.lateral_offsets_m, baseline.lateral_offsets_m);
  EXPECT_EQ(output.speed_caps_mps, baseline.speed_caps_mps);
  EXPECT_EQ(output.longitudinal_offsets_m, baseline.longitudinal_offsets_m);
  EXPECT_EQ(output.preventive_side_role_active,
            baseline.preventive_side_role_active);
  EXPECT_FALSE(output.preventive_side_role_active);
  EXPECT_FALSE(output.pass_continuation_active);
  EXPECT_TRUE(output.early_aware_diagnostic.active);
  EXPECT_FALSE(baseline.early_aware_diagnostic.active);
}

TEST(LatticePlanner, SafeStopClearsEarlyAwareDiagnosticAfterSelectorUpdate) {
  sl::PlannerConfig observed_config;
  observed_config.normal_mode_min_hold_sec = 0.0;
  observed_config.preventive_side_role_trigger_clearance_m =
      observed_config.opponent_hard_clearance_m + 1.0e-3;
  observed_config.stop_cost = 0;
  auto disabled_config = observed_config;
  disabled_config.early_aware_enabled = false;
  auto frame = straightFrame();
  auto map = openGrid(observed_config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, observed_config));
  sl::EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 6.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  sl::OpponentState target;
  target.id = "early_safe_stop";
  target.x = 14.0;
  target.y = 0.0;
  target.yaw = 0.0;
  target.vx_mps = 0.0;
  target.vy_mps = 0.0;
  target.stamp_sec = 10.0;
  target.frenet = frame.project(target.x, target.y, target.yaw);
  target.valid = target.frenet.valid;
  sl::LatticePlanner observed(observed_config, &frame, &map);
  sl::LatticePlanner disabled(disabled_config, &frame, &map);
  (void)observed.update(ego, {target}, true, 10.0);
  (void)disabled.update(ego, {target}, true, 10.0);

  ego.stamp_sec = 10.1;
  target.stamp_sec = 10.1;
  const auto output = observed.update(ego, {target}, true, 10.1);
  const auto baseline = disabled.update(ego, {target}, true, 10.1);

  EXPECT_EQ(output.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(output.mode, baseline.mode);
  EXPECT_EQ(output.active, baseline.active);
  EXPECT_EQ(output.intent, baseline.intent);
  EXPECT_EQ(output.safe_lateral, baseline.safe_lateral);
  EXPECT_EQ(output.emergency_stop, baseline.emergency_stop);
  EXPECT_EQ(output.speed_cap_mps, baseline.speed_cap_mps);
  EXPECT_EQ(output.lateral_offsets_m, baseline.lateral_offsets_m);
  EXPECT_EQ(output.speed_caps_mps, baseline.speed_caps_mps);
  EXPECT_EQ(output.longitudinal_offsets_m, baseline.longitudinal_offsets_m);
  EXPECT_FALSE(output.early_aware_diagnostic.evaluated);
  EXPECT_FALSE(output.early_aware_diagnostic.active);
  EXPECT_EQ(output.early_aware_diagnostic.state, sl::EarlyAwareState::INACTIVE);
  EXPECT_EQ(output.early_aware_diagnostic.reason, "reset");
  EXPECT_FALSE(observed.earlyAwareSelector().diagnostic().active);
  EXPECT_EQ(observed.earlyAwareSelector().diagnostic().state,
            sl::EarlyAwareState::INACTIVE);
}

TEST(FrontDetector, ReportsPerOpponentFirstFalseWithoutChangingSelection) {
  const sl::PlannerConfig config;
  auto frame = sampledSquareFrame();
  sl::EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 3.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;

  sl::OpponentState rear;
  rear.id = "rear";
  rear.x = 2.0;
  rear.y = 0.0;
  rear.yaw = 0.0;
  rear.frenet = frame.project(rear.x, rear.y, rear.yaw);
  rear.valid = rear.frenet.valid;

  sl::OpponentState outside_transition = rear;
  outside_transition.id = "outside_transition";
  outside_transition.x = 8.0;
  outside_transition.y = 8.0;
  outside_transition.frenet = frame.project(
      outside_transition.x, outside_transition.y, outside_transition.yaw);
  outside_transition.valid = outside_transition.frenet.valid;

  sl::FrontDetector detector(config);
  EXPECT_FALSE(
      detector.update(ego, {rear, outside_transition}, frame).has_value());
  const auto &diagnostic = detector.diagnostic();
  ASSERT_EQ(diagnostic.opponent_count, 2U);
  EXPECT_STREQ(diagnostic.opponents[0].opponent_id.data(), "rear");
  EXPECT_EQ(diagnostic.opponents[0].first_false,
            sl::FrontDetectionFailure::LONGITUDINAL_TARGET);
  EXPECT_STREQ(diagnostic.opponents[1].opponent_id.data(),
               "outside_transition");
  EXPECT_EQ(diagnostic.opponents[1].first_false,
            sl::FrontDetectionFailure::TRANSITION_ENVELOPE);
}

TEST(FrontDetector, BoundedDiagnosticsDoNotChangeFullTargetIdentity) {
  const sl::PlannerConfig config;
  auto frame = sampledSquareFrame();
  sl::EgoState ego;
  ego.x = 5.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 3.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;

  std::vector<sl::OpponentState> opponents(10U);
  for (std::size_t i = 0U; i < opponents.size(); ++i) {
    opponents[i].id = "invalid_" + std::to_string(i);
    opponents[i].valid = false;
  }
  sl::OpponentState forward;
  forward.id = "forward_target_with_a_diagnostic_id_longer_than_31_bytes";
  forward.x = 8.0;
  forward.y = 0.0;
  forward.yaw = 0.0;
  forward.frenet = frame.project(forward.x, forward.y, forward.yaw);
  forward.valid = forward.frenet.valid;
  opponents.push_back(forward);

  sl::FrontDetector detector(config);
  const auto selected = detector.update(ego, opponents, frame);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected.value(), forward.id);
  const auto &diagnostic = detector.diagnostic();
  EXPECT_TRUE(diagnostic.evaluated);
  EXPECT_EQ(diagnostic.opponent_count,
            sl::kMaxFrontDetectionOpponentDiagnostics);
  EXPECT_EQ(diagnostic.dropped_opponent_count, 3U);
  EXPECT_EQ(
      std::char_traits<char>::length(diagnostic.selected_target_id.data()),
      sl::kFrontDetectionDiagnosticIdCapacity - 1U);
}

TEST(FrontDetector, Dev3D3PreHardFixtureReportsExactDetectorFailure) {
  const sl::PlannerConfig config;
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;

  const auto &fixture = sl::test_fixture::kD3PreWallFreeRun;
  sl::EgoState ego;
  ego.x = fixture.ego.x_m;
  ego.y = fixture.ego.y_m;
  ego.yaw = fixture.ego.yaw_rad;
  ego.speed_mps = fixture.ego.speed_mps;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  std::vector<sl::OpponentState> opponents;
  for (const auto &source : fixture.opponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.frenet = frame.project(opponent.x, opponent.y, 0.0);
    ASSERT_TRUE(opponent.frenet.valid);
    opponent.yaw = frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.uncertainty_x_m = fixture.opponent_uncertainty_x_m;
    opponent.uncertainty_y_m = fixture.opponent_uncertainty_y_m;
    opponent.valid = opponent.frenet.valid;
    opponents.push_back(opponent);
  }

  sl::FrontDetector detector(config);
  EXPECT_FALSE(detector.update(ego, opponents, frame).has_value());
  const auto &diagnostic = detector.diagnostic();
  ASSERT_EQ(diagnostic.opponent_count, 2U);
  const auto d2 =
      std::find_if(diagnostic.opponents.begin(),
                   diagnostic.opponents.begin() + diagnostic.opponent_count,
                   [](const sl::FrontDetectionOpponentDiagnostic &entry) {
                     return std::string_view(entry.opponent_id.data()) == "d2";
                   });
  ASSERT_NE(d2, diagnostic.opponents.begin() + diagnostic.opponent_count);
  // D2 is physically close in Cartesian space, but it is behind D3 in the
  // track-relative frame and outside the existing parallel rear envelope.
  // The detector therefore rejects it before lateral sweep evaluation.
  EXPECT_EQ(d2->first_false, sl::FrontDetectionFailure::LONGITUDINAL_TARGET);
  EXPECT_EQ(d2->target_class, sl::FrontTargetClass::NONE);
  EXPECT_FALSE(std::isfinite(d2->minimum_sweep_distance_m));
}

TEST(LatticePlanner, Dev3D3PreHardParallelFixtureStartsCandidateGeneration) {
  sl::PlannerConfig config;
  // Preserve this fixture as a detector/candidate-generation regression. The
  // preventive role guard is validated independently with resolved IDs.
  config.preventive_side_role_trigger_clearance_m =
      config.opponent_hard_clearance_m + 1.0e-3;
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  const auto &fixture = sl::test_fixture::kD3PreHardParallel;
  sl::EgoState ego;
  ego.x = fixture.ego.x_m;
  ego.y = fixture.ego.y_m;
  ego.yaw = fixture.ego.yaw_rad;
  ego.speed_mps = fixture.ego.speed_mps;
  ego.stamp_sec = 100.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  std::vector<sl::OpponentState> opponents;
  for (const auto &source : fixture.opponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.frenet = frame.project(opponent.x, opponent.y, 0.0);
    ASSERT_TRUE(opponent.frenet.valid);
    opponent.yaw = frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.stamp_sec = 100.0;
    opponent.uncertainty_x_m = fixture.opponent_uncertainty_x_m;
    opponent.uncertainty_y_m = fixture.opponent_uncertainty_y_m;
    opponent.valid = opponent.frenet.valid;
    opponents.push_back(opponent);
  }
  sl::LatticePlanner planner(config, &frame, &map);
  const auto output = planner.update(ego, opponents, true, 100.0);
  EXPECT_NE(output.reason, "current_pose_hard_collision");
  EXPECT_NE(output.mode, sl::BehaviorMode::FREE_RUN);
  EXPECT_EQ(output.target_id, fixture.expected_target_id);
  EXPECT_GT(output.generated_candidates, 0);
}

TEST(LatticePlanner, Dev3CurrentPoseFixturesReportOpponentAndClearance) {
  const sl::PlannerConfig config;
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  for (const auto &fixture : sl::test_fixture::kCurrentClearance) {
    sl::LatticePlanner planner(config, &frame, &map);
    sl::EgoState ego;
    ego.x = fixture.ego.x_m;
    ego.y = fixture.ego.y_m;
    ego.yaw = fixture.ego.yaw_rad;
    ego.speed_mps = fixture.ego.speed_mps;
    ego.stamp_sec = 100.0;
    ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
    ego.valid = ego.frenet.valid;
    ASSERT_TRUE(ego.valid) << fixture.ego.vehicle_id;
    sl::OpponentState opponent;
    opponent.id = std::string(fixture.opponent.vehicle_id);
    opponent.x = fixture.opponent.x_m;
    opponent.y = fixture.opponent.y_m;
    opponent.yaw = fixture.opponent.yaw_rad;
    opponent.speed_mps = fixture.opponent.speed_mps;
    opponent.uncertainty_x_m = fixture.opponent_uncertainty_x_m;
    opponent.uncertainty_y_m = fixture.opponent_uncertainty_y_m;
    opponent.stamp_sec = 100.0;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.valid = opponent.frenet.valid;
    ASSERT_TRUE(opponent.valid) << fixture.opponent.vehicle_id;

    const auto output = planner.update(ego, {opponent}, true, 100.0);
    EXPECT_EQ(output.reason, "current_pose_hard_collision");
    EXPECT_EQ(output.current_collision_kind, sl::CollisionKind::OPPONENT);
    EXPECT_EQ(output.blocking_opponent_id, fixture.opponent.vehicle_id);
    EXPECT_NEAR(output.blocking_clearance_m, fixture.expected_clearance_m, 0.03)
        << fixture.ego.vehicle_id;
    EXPECT_DOUBLE_EQ(output.blocking_required_clearance_m,
                     config.opponent_hard_clearance_m);
  }
}

TEST(LatticePlanner, Dev3D1FollowFixtureReportsCandidateRejectBuckets) {
  sl::PlannerConfig config;
  // Preserve the historical run's loaded limits so this fixture continues to
  // prove the original first-reject distribution.
  config.hard_max_steer_rad = 0.5585;
  config.planner_max_steer_rad = 0.5236;
  config.max_steer_rate_radps = 0.35;
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  const auto &fixture = sl::test_fixture::kD1FollowBlocked;
  sl::EgoState ego;
  ego.x = fixture.ego.x_m;
  ego.y = fixture.ego.y_m;
  ego.yaw = fixture.ego.yaw_rad;
  ego.speed_mps = fixture.ego.speed_mps;
  ego.stamp_sec = 100.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);
  std::vector<sl::OpponentState> opponents;
  for (const auto &source : fixture.opponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.frenet = frame.project(opponent.x, opponent.y, 0.0);
    ASSERT_TRUE(opponent.frenet.valid);
    opponent.yaw = frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.uncertainty_x_m = fixture.opponent_uncertainty_x_m;
    opponent.uncertainty_y_m = fixture.opponent_uncertainty_y_m;
    opponent.stamp_sec = 100.0;
    opponent.valid = opponent.frenet.valid;
    opponents.push_back(opponent);
  }

  const auto output = planner.update(ego, opponents, true, 100.0);
  EXPECT_EQ(output.target_id, "d3");
  EXPECT_EQ(output.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_GT(output.speed_cap_mps, config.safe_stop_speed_mps);
  EXPECT_EQ(output.feasible_candidates, fixture.observed_feasible_candidates);
  EXPECT_EQ(output.generated_candidates, 21);
  EXPECT_EQ(output.rejected_wall_candidates +
                output.rejected_opponent_candidates +
                output.rejected_curvature_candidates +
                output.rejected_trackability_candidates +
                output.rejected_other_candidates,
            output.generated_candidates);
  EXPECT_EQ(output.rejected_wall_candidates, fixture.expected_wall_rejections);
  EXPECT_EQ(output.rejected_opponent_candidates,
            fixture.expected_opponent_rejections);
  EXPECT_EQ(output.rejected_curvature_candidates,
            fixture.expected_curvature_rejections);
  EXPECT_EQ(output.rejected_trackability_candidates,
            fixture.expected_trackability_rejections);
  EXPECT_EQ(output.rejected_other_candidates,
            fixture.expected_other_rejections);

  // Re-evaluate the exact same pose/opponents with the PP/Mux limits used by
  // the State Lattice + Pure Pursuit profile. Wall and all-opponent checks stay
  // enabled; only the stale proxy limits differ.
  sl::PlannerConfig pp_aligned_config = config;
  pp_aligned_config.hard_max_steer_rad = 0.64;
  pp_aligned_config.planner_max_steer_rad = 0.64;
  pp_aligned_config.max_steer_rate_radps = 128.0;
  sl::LatticePlanner pp_aligned_planner(pp_aligned_config, &frame, &map);
  const auto pp_aligned_output =
      pp_aligned_planner.update(ego, opponents, true, 100.0);
  EXPECT_EQ(pp_aligned_output.generated_candidates,
            output.generated_candidates);
  EXPECT_LT(pp_aligned_output.rejected_trackability_candidates,
            output.rejected_trackability_candidates);
  EXPECT_EQ(pp_aligned_output.rejected_wall_candidates +
                pp_aligned_output.rejected_opponent_candidates +
                pp_aligned_output.rejected_curvature_candidates +
                pp_aligned_output.rejected_trackability_candidates +
                pp_aligned_output.rejected_other_candidates +
                pp_aligned_output.feasible_candidates,
            pp_aligned_output.generated_candidates);
}

TEST(ParametricQuintic,
     Dev3D1ClearanceRecoverySeparatesCompressedArcFromDiscontinuity) {
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;

  // Nearest kinematic-state sample (1.824 ms after generation 917) for the
  // D1 first-false stamp 1785249433677473867 in
  // output/20260728-side-clearance-class-v1.
  const sl::Pose2d start{89630.7975813119, 43129.0535720292, 1.89510039373321};
  const double start_speed_mps = 1.33691961563367;
  const double start_yaw_rate_radps = -0.184349668466327;
  const double start_curvature = start_yaw_rate_radps / start_speed_mps;
  const auto start_frenet = frame.project(start.x, start.y, start.yaw);
  ASSERT_TRUE(start_frenet.valid);

  constexpr double goal_d_m = 2.2;
  constexpr double compressed_arc_m = 1.0;
  constexpr double nominal_arc_m = 4.273;
  const sl::PlannerConfig config;
  const double curvature_limit = std::tan(std::min(config.planner_max_steer_rad,
                                                   config.hard_max_steer_rad)) /
                                 config.wheel_base_m;

  const auto maximumCurvature = [&](double arc_m, double tangent_scale) {
    const auto reference_goal = frame.interpolate(start_frenet.s + arc_m);
    const auto goal_point =
        frame.frenetToCartesian(start_frenet.s + arc_m, goal_d_m);
    const sl::Pose2d goal{goal_point.x, goal_point.y, reference_goal.yaw};
    const double denominator = 1.0 - goal_d_m * reference_goal.kappa;
    EXPECT_GT(denominator, 1.0e-3);
    const double goal_curvature = reference_goal.kappa / denominator;
    sl::ParametricQuintic polynomial;
    EXPECT_TRUE(polynomial.configure(start, start_curvature, goal,
                                     goal_curvature, tangent_scale));

    double maximum_abs_curvature = 0.0;
    double maximum_u = 0.0;
    double minimum_step_m = std::numeric_limits<double>::infinity();
    auto previous = polynomial.sample(0.0);
    EXPECT_NEAR(previous.kappa, start_curvature, 1.0e-9);
    constexpr int samples = 10000;
    for (int i = 1; i <= samples; ++i) {
      const double u = static_cast<double>(i) / samples;
      const auto point = polynomial.sample(u);
      EXPECT_TRUE(std::isfinite(point.x));
      EXPECT_TRUE(std::isfinite(point.y));
      EXPECT_TRUE(std::isfinite(point.yaw));
      EXPECT_TRUE(std::isfinite(point.kappa));
      minimum_step_m =
          std::min(minimum_step_m,
                   std::hypot(point.x - previous.x, point.y - previous.y));
      if (std::abs(point.kappa) > maximum_abs_curvature) {
        maximum_abs_curvature = std::abs(point.kappa);
        maximum_u = u;
      }
      previous = point;
    }
    EXPECT_NEAR(previous.kappa, goal_curvature, 1.0e-8);
    EXPECT_GT(minimum_step_m, 0.0);
    EXPECT_GT(maximum_u, 0.0);
    EXPECT_LT(maximum_u, 1.0);
    return maximum_abs_curvature;
  };

  for (const double tangent_scale : {0.8, 1.0, 1.2}) {
    EXPECT_GT(maximumCurvature(compressed_arc_m, tangent_scale),
              curvature_limit);
    EXPECT_LT(maximumCurvature(nominal_arc_m, tangent_scale), curvature_limit);
  }
}

TEST(LatticePlanner,
     Dev3D1EarlyTransitionUsesFullArcOnlyBeforeObstacleDeadline) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.2, -2.2};
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  sl::EgoState ego;
  ego.x = 89630.806;
  ego.y = 43129.026;
  ego.yaw = 1.900;
  ego.speed_mps = 1.33691961563367;
  ego.yaw_rate_radps = -0.184349668466327;
  ego.curvature = ego.yaw_rate_radps / ego.speed_mps;
  ego.stamp_sec = 100.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  const std::size_t nearest = frame.nearestIndex(ego.frenet.s);
  const double midpoint_s =
      0.5 * (frame.unwrappedIndexS(static_cast<long long>(nearest), nearest,
                                   ego.frenet.s) +
             frame.unwrappedIndexS(static_cast<long long>(nearest) +
                                       config.mpc_wp_id_offset,
                                   nearest, ego.frenet.s));
  const double furthest_receiver_s = frame.unwrappedIndexS(
      static_cast<long long>(nearest) + config.mpc_wp_id_offset +
          config.nearest_index_uncertainty,
      nearest, ego.frenet.s);
  const double receiver_alignment_lead =
      std::max(0.0, furthest_receiver_s - midpoint_s);
  const double pre_obstacle_separation =
      config.wheel_base_m + config.front_overhang_m +
      config.opponent_longitudinal_tracking_margin_m + config.rear_overhang_m +
      config.opponent_hard_clearance_m + config.pass_lateral_extra_margin_m;
  const double required_arc_m = sl::requiredLateralTransitionDistance(
      config, ego.speed_mps, ego.frenet.d, 2.2);

  const auto opponentAtAvailableArc = [&](double available_arc_m) {
    sl::OpponentState opponent;
    opponent.id = "d3";
    const double opponent_s = ego.frenet.s + pre_obstacle_separation +
                              receiver_alignment_lead + available_arc_m;
    const auto pose = frame.frenetToCartesian(opponent_s, 0.6);
    opponent.x = pose.x;
    opponent.y = pose.y;
    opponent.yaw = pose.yaw;
    opponent.speed_mps = 0.5;
    opponent.vx_mps = opponent.speed_mps * std::cos(opponent.yaw);
    opponent.vy_mps = opponent.speed_mps * std::sin(opponent.yaw);
    opponent.stamp_sec = 100.0;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.valid = opponent.frenet.valid;
    return opponent;
  };

  {
    sl::LatticePlanner late_planner(config, &frame, &map);
    const auto output =
        late_planner.update(ego, {opponentAtAvailableArc(1.0)}, true, 100.0);
    ASSERT_FALSE(late_planner.candidates().empty()) << output.reason;
    int recovery_candidates = 0;
    for (const auto &candidate : late_planner.candidates()) {
      if (candidate.lateral_index != 3U) {
        continue;
      }
      ++recovery_candidates;
      EXPECT_NEAR(candidate.goal_d_m, 2.2, 1.0e-6);
      EXPECT_NEAR(candidate.required_arc_m,
                  config.minimum_obstacle_transition_distance_m, 1.0e-6);
      EXPECT_EQ(candidate.rejection_reason, "maximum_curvature");
    }
    EXPECT_EQ(recovery_candidates, 3);
    EXPECT_NE(output.reason, "current_pose_hard_collision");
  }

  {
    sl::LatticePlanner early_planner(config, &frame, &map);
    const auto output = early_planner.update(
        ego, {opponentAtAvailableArc(required_arc_m + 2.0)}, true, 100.0);
    ASSERT_FALSE(early_planner.candidates().empty()) << output.reason;
    int recovery_candidates = 0;
    int curvature_safe_candidates = 0;
    const double curvature_limit =
        std::tan(
            std::min(config.planner_max_steer_rad, config.hard_max_steer_rad)) /
        config.wheel_base_m;
    for (const auto &candidate : early_planner.candidates()) {
      if (candidate.lateral_index != 3U) {
        continue;
      }
      ++recovery_candidates;
      EXPECT_NEAR(candidate.goal_d_m, 2.2, 1.0e-6);
      EXPECT_NEAR(candidate.required_arc_m, required_arc_m, 1.0e-6);
      EXPECT_NE(candidate.rejection_reason, "maximum_curvature");
      double maximum_abs_curvature = 0.0;
      for (const auto &point : candidate.dense) {
        ASSERT_TRUE(std::isfinite(point.kappa));
        maximum_abs_curvature =
            std::max(maximum_abs_curvature, std::abs(point.kappa));
      }
      if (!candidate.dense.empty() &&
          maximum_abs_curvature <= curvature_limit + 1.0e-6) {
        ++curvature_safe_candidates;
      }
    }
    EXPECT_EQ(recovery_candidates, 3);
    EXPECT_EQ(curvature_safe_candidates, 3);
    EXPECT_NE(output.reason, "current_pose_hard_collision");
  }
}

TEST(LatticePlanner, Dev3D2OutputContractFixtureUsesForwardExactHorizon) {
  sl::PlannerConfig config;
  // Preserve the limits loaded by this historical bag-derived fixture. The
  // PP-aligned defaults are covered independently by the D1 comparison above.
  config.hard_max_steer_rad = 0.5585;
  config.planner_max_steer_rad = 0.5236;
  config.max_steer_rate_radps = 0.35;
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  const auto &fixture = sl::test_fixture::kD2FirstOutputContract;
  sl::EgoState ego;
  ego.x = fixture.ego.x_m;
  ego.y = fixture.ego.y_m;
  ego.yaw = fixture.ego.yaw_rad;
  ego.speed_mps = fixture.ego.speed_mps;
  ego.stamp_sec = 100.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  std::vector<sl::OpponentState> opponents;
  for (const auto &source : fixture.opponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.frenet = frame.project(opponent.x, opponent.y, 0.0);
    ASSERT_TRUE(opponent.frenet.valid);
    opponent.yaw = frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.speed_mps = source.speed_mps;
    opponent.stamp_sec = 100.0;
    opponent.uncertainty_x_m = fixture.opponent_uncertainty_x_m;
    opponent.uncertainty_y_m = fixture.opponent_uncertainty_y_m;
    opponent.valid = opponent.frenet.valid;
    opponents.push_back(opponent);
  }

  sl::LatticePlanner planner(config, &frame, &map);
  const auto output = planner.update(ego, opponents, true, 100.0);
  EXPECT_EQ(output.generated_candidates, fixture.observed_generated_candidates);
  EXPECT_EQ(fixture.bag_observed_feasible_candidates, 16);
  EXPECT_EQ(output.feasible_candidates,
            fixture.replay_expected_feasible_candidates);
  EXPECT_EQ(output.reason, "selected_right_candidate");
  EXPECT_EQ(output.mode, sl::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_TRUE(output.active);
  EXPECT_TRUE(output.safe_lateral);
  EXPECT_FALSE(output.emergency_stop);
  EXPECT_FALSE(output.lateral_offsets_m.empty());
  EXPECT_EQ(output.lateral_offsets_m.size(), output.speed_caps_mps.size());
  EXPECT_EQ(output.lateral_offsets_m.size(),
            output.longitudinal_offsets_m.size());
  ASSERT_FALSE(output.longitudinal_offsets_m.empty());
  EXPECT_DOUBLE_EQ(output.longitudinal_offsets_m.front(), 0.0);
  EXPECT_TRUE(std::is_sorted(output.longitudinal_offsets_m.begin(),
                             output.longitudinal_offsets_m.end()));
  EXPECT_TRUE(output.spatial_profile_shadow_only);
  const auto first_forward =
      std::find_if(output.longitudinal_offsets_m.begin(),
                   output.longitudinal_offsets_m.end(),
                   [](double offset_m) { return offset_m > 1.0e-6; });
  ASSERT_NE(first_forward, output.longitudinal_offsets_m.end());
  const auto first_forward_index = static_cast<std::size_t>(
      std::distance(output.longitudinal_offsets_m.begin(), first_forward));
  EXPECT_DOUBLE_EQ(output.speed_cap_mps,
                   output.speed_caps_mps[first_forward_index]);
  EXPECT_TRUE(planner.selected().has_value());
  EXPECT_EQ(planner.candidates().size(), 21U);

  EXPECT_STREQ(sl::toString(output.output_horizon_diagnostic.failure), "none");
  EXPECT_EQ(output.output_horizon_diagnostic.nearest_shift, 0);
  EXPECT_EQ(output.output_horizon_diagnostic.layout_offset, 0);
  EXPECT_EQ(output.output_horizon_diagnostic.waypoint_index, -1);
  EXPECT_EQ(output.output_horizon_diagnostic.interpolation_piece, -1);
  EXPECT_EQ(output.output_horizon_diagnostic.reference_index, -1);
  EXPECT_TRUE(std::isnan(output.output_horizon_diagnostic.candidate_goal_d_m));
  EXPECT_TRUE(
      std::isnan(output.output_horizon_diagnostic.candidate_tangent_scale));
  EXPECT_TRUE(std::isnan(output.output_horizon_diagnostic.observed_value));
  EXPECT_TRUE(std::isnan(output.output_horizon_diagnostic.limit_value));
  EXPECT_TRUE(output.output_horizon_diagnostic.opponent_id.empty());

  const auto &metrics = planner.lastPlanningCycleMetrics();
  EXPECT_TRUE(std::isfinite(metrics.candidate_generation_ms));
  EXPECT_TRUE(std::isfinite(metrics.output_horizon_ms));
  EXPECT_GE(metrics.candidate_generation_ms, 0.0);
  EXPECT_GE(metrics.output_horizon_ms, 0.0);
  EXPECT_GT(metrics.candidate_dense_point_count, 0U);
  EXPECT_GT(metrics.output_horizon_call_count, 0U);
  ASSERT_EQ(metrics.candidate_diagnostic_count, planner.candidates().size());
  for (std::size_t i = 0U; i < planner.candidates().size(); ++i) {
    const auto &candidate = planner.candidates()[i];
    const auto &diagnostic = metrics.candidate_diagnostics[i];
    EXPECT_EQ(diagnostic.lateral_index, candidate.lateral_index);
    EXPECT_EQ(diagnostic.tangent_index, candidate.tangent_index);
    EXPECT_DOUBLE_EQ(diagnostic.goal_d_m, candidate.goal_d_m);
    EXPECT_DOUBLE_EQ(diagnostic.tangent_scale, candidate.tangent_scale);
    EXPECT_DOUBLE_EQ(diagnostic.required_arc_m, candidate.required_arc_m);
    EXPECT_EQ(diagnostic.total_cost, candidate.total_cost);
    EXPECT_EQ(diagnostic.reference_cost, candidate.reference_cost);
    EXPECT_EQ(diagnostic.wall_cost, candidate.wall_cost);
    EXPECT_EQ(diagnostic.object_cost, candidate.object_cost);
    EXPECT_EQ(diagnostic.representative_wall_diagnostic_valid,
              candidate.representative_wall_diagnostic_valid);
    EXPECT_EQ(
        diagnostic.representative_minimum_nominal_wall_clearance_point_index,
        candidate.representative_minimum_nominal_wall_clearance_point_index);
    EXPECT_EQ(diagnostic.representative_maximum_nominal_wall_level,
              candidate.representative_maximum_nominal_wall_level);
    EXPECT_EQ(diagnostic.representative_maximum_nominal_wall_level_point_index,
              candidate.representative_maximum_nominal_wall_level_point_index);
    EXPECT_EQ(diagnostic.representative_object_diagnostic_count,
              candidate.representative_object_diagnostic_count);
    EXPECT_LE(diagnostic.representative_object_diagnostic_count,
              diagnostic.representative_object_diagnostics.size());
    for (std::size_t representative_index = 0U;
         representative_index <
         diagnostic.representative_object_diagnostic_count;
         ++representative_index) {
      const auto &actual =
          diagnostic.representative_object_diagnostics[representative_index];
      const auto &expected =
          candidate.representative_object_diagnostics[representative_index];
      EXPECT_EQ(actual.winner_valid, expected.winner_valid);
      EXPECT_EQ(actual.prediction_valid, expected.prediction_valid);
      EXPECT_EQ(actual.object_level, expected.object_level);
      EXPECT_EQ(std::string(actual.opponent_id.data()),
                std::string(expected.opponent_id.data()));
      EXPECT_EQ(actual.opponent_id_truncated, expected.opponent_id_truncated);
      EXPECT_DOUBLE_EQ(actual.prediction_horizon_sec,
                       expected.prediction_horizon_sec);
      if (expected.prediction_valid) {
        EXPECT_TRUE(std::isfinite(actual.clearance_m));
        EXPECT_DOUBLE_EQ(actual.clearance_m, expected.clearance_m);
      } else {
        EXPECT_TRUE(std::isnan(actual.clearance_m));
      }
      if (!expected.winner_valid) {
        EXPECT_TRUE(std::string(actual.opponent_id.data()).empty());
      }
    }
    if (candidate.representative_wall_diagnostic_valid) {
      EXPECT_TRUE(std::isfinite(
          diagnostic.representative_minimum_nominal_wall_clearance_proxy_m));
      EXPECT_DOUBLE_EQ(
          diagnostic.representative_minimum_nominal_wall_clearance_proxy_m,
          candidate.representative_minimum_nominal_wall_clearance_proxy_m);
      EXPECT_GE(
          diagnostic.representative_minimum_nominal_wall_clearance_point_index,
          0);
      EXPECT_GE(diagnostic.representative_maximum_nominal_wall_level, 0);
      EXPECT_GE(
          diagnostic.representative_maximum_nominal_wall_level_point_index, 0);
    } else {
      EXPECT_TRUE(std::isnan(
          diagnostic.representative_minimum_nominal_wall_clearance_proxy_m));
      EXPECT_EQ(
          diagnostic.representative_minimum_nominal_wall_clearance_point_index,
          -1);
    }
    EXPECT_GE(diagnostic.reference_cost, 0);
    EXPECT_GE(diagnostic.wall_cost, 0);
    EXPECT_GE(diagnostic.object_cost, 0);
    EXPECT_DOUBLE_EQ(diagnostic.start_x_m, ego.x);
    EXPECT_DOUBLE_EQ(diagnostic.start_y_m, ego.y);
    EXPECT_DOUBLE_EQ(diagnostic.start_yaw_rad, ego.yaw);
    EXPECT_DOUBLE_EQ(diagnostic.start_s_m, ego.frenet.s);
    EXPECT_DOUBLE_EQ(diagnostic.start_d_m, ego.frenet.d);
    EXPECT_EQ(std::string(diagnostic.first_reject_reason.data()),
              candidate.rejection_reason.empty() ? "feasible"
                                                 : candidate.rejection_reason);
  }
}

TEST(LatticePlanner,
     Dev3D2TrackabilityFixturePreservesPerCandidateFirstRejectEvidence) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  const auto &fixture = sl::test_fixture::kD2FirstTrackabilityReject;
  sl::EgoState ego;
  ego.x = fixture.ego.x_m;
  ego.y = fixture.ego.y_m;
  ego.yaw = fixture.ego.yaw_rad;
  ego.speed_mps = fixture.ego.speed_mps;
  ego.stamp_sec = 100.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  std::vector<sl::OpponentState> opponents;
  for (const auto &source : fixture.opponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.frenet = frame.project(opponent.x, opponent.y, source.yaw_rad);
    ASSERT_TRUE(opponent.frenet.valid);
    opponent.yaw = frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.speed_mps = source.speed_mps;
    opponent.stamp_sec = 100.0;
    opponent.uncertainty_x_m = fixture.opponent_uncertainty_x_m;
    opponent.uncertainty_y_m = fixture.opponent_uncertainty_y_m;
    opponent.valid = opponent.frenet.valid;
    opponents.push_back(opponent);
  }

  sl::LatticePlanner planner(config, &frame, &map);
  const auto output = planner.update(ego, opponents, true, 100.0);
  EXPECT_EQ(output.generated_candidates, fixture.observed_generated_candidates);
  EXPECT_EQ(fixture.bag_observed_feasible_candidates, 2);
  EXPECT_EQ(output.feasible_candidates,
            fixture.replay_expected_feasible_candidates);
  EXPECT_EQ(output.rejected_wall_candidates, fixture.observed_wall_rejections);
  EXPECT_EQ(output.rejected_opponent_candidates,
            fixture.observed_opponent_rejections);
  EXPECT_EQ(output.rejected_trackability_candidates,
            fixture.replay_expected_trackability_rejections);
  EXPECT_EQ(fixture.bag_observed_trackability_rejections, 1);

  const auto &metrics = planner.lastPlanningCycleMetrics();
  ASSERT_EQ(metrics.candidate_diagnostic_count, planner.candidates().size());
  std::size_t trackability_diagnostic_count = 0U;
  for (std::size_t i = 0U; i < planner.candidates().size(); ++i) {
    const auto &candidate = planner.candidates()[i];
    const auto &diagnostic = metrics.candidate_diagnostics[i];
    EXPECT_DOUBLE_EQ(diagnostic.goal_d_m, candidate.goal_d_m);
    EXPECT_DOUBLE_EQ(diagnostic.tangent_scale, candidate.tangent_scale);
    EXPECT_DOUBLE_EQ(diagnostic.required_arc_m, candidate.required_arc_m);
    EXPECT_DOUBLE_EQ(diagnostic.start_x_m, ego.x);
    EXPECT_DOUBLE_EQ(diagnostic.start_y_m, ego.y);
    EXPECT_DOUBLE_EQ(diagnostic.start_yaw_rad, ego.yaw);
    EXPECT_DOUBLE_EQ(diagnostic.start_s_m, ego.frenet.s);
    EXPECT_DOUBLE_EQ(diagnostic.start_d_m, ego.frenet.d);
    const std::string first_reject_reason{
        diagnostic.first_reject_reason.data()};
    EXPECT_EQ(first_reject_reason, candidate.rejection_reason.empty()
                                       ? "feasible"
                                       : candidate.rejection_reason);
    if (first_reject_reason == "entry_speed_exceeds_candidate_limit" ||
        first_reject_reason == "frenet_projection" ||
        first_reject_reason == "representative_projection" ||
        first_reject_reason == "zero_length_steering_change" ||
        first_reject_reason == "steering_rate_below_safe_stop" ||
        first_reject_reason == "deceleration_profile") {
      ++trackability_diagnostic_count;
    }
  }
  EXPECT_EQ(trackability_diagnostic_count,
            static_cast<std::size_t>(
                fixture.replay_expected_trackability_rejections));
}

TEST(LatticePlanner, Dev3D2ExactCartesianHorizonNeverStartsBehindEgo) {
  sl::PlannerConfig config;
  // Diagnostic-only contract probe: an exact reference source fixes both the
  // receiver nearest index and the PP/MPC layout. The live V3 wire does not
  // provide this guarantee and therefore keeps the production defaults above.
  config.nearest_index_uncertainty = 0;
  config.mpc_wp_id_offset = 0;
  config.experimental_exact_spatial_follow_shadow_enabled = true;

  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  const auto &fixture = sl::test_fixture::kD2FirstOutputContract;
  sl::EgoState ego;
  ego.x = fixture.ego.x_m;
  ego.y = fixture.ego.y_m;
  ego.yaw = fixture.ego.yaw_rad;
  ego.speed_mps = fixture.ego.speed_mps;
  ego.stamp_sec = 100.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  std::vector<sl::OpponentState> opponents;
  for (const auto &source : fixture.opponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.frenet = frame.project(opponent.x, opponent.y, 0.0);
    ASSERT_TRUE(opponent.frenet.valid);
    opponent.yaw = frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.speed_mps = source.speed_mps;
    opponent.stamp_sec = 100.0;
    opponent.uncertainty_x_m = fixture.opponent_uncertainty_x_m;
    opponent.uncertainty_y_m = fixture.opponent_uncertainty_y_m;
    opponent.valid = opponent.frenet.valid;
    opponents.push_back(opponent);
  }

  sl::LatticePlanner planner(config, &frame, &map);
  const auto output = planner.update(ego, opponents, true, 100.0);

  ASSERT_GT(output.feasible_candidates, 0);
  const auto feasible =
      std::find_if(planner.candidates().begin(), planner.candidates().end(),
                   [](const sl::CandidateTrajectory &candidate) {
                     return candidate.feasible && !candidate.dense.empty();
                   });
  ASSERT_NE(feasible, planner.candidates().end());
  EXPECT_NEAR(feasible->dense.front().x, ego.x, 1.0e-6);
  EXPECT_NEAR(feasible->dense.front().y, ego.y, 1.0e-6);

  EXPECT_TRUE(output.safe_lateral);
  EXPECT_FALSE(output.lateral_offsets_m.empty());
  EXPECT_EQ(output.lateral_offsets_m.size(), output.speed_caps_mps.size());
  EXPECT_EQ(output.lateral_offsets_m.size(),
            output.longitudinal_offsets_m.size());
  ASSERT_FALSE(output.longitudinal_offsets_m.empty());
  EXPECT_DOUBLE_EQ(output.longitudinal_offsets_m.front(), 0.0);
  EXPECT_TRUE(std::is_sorted(output.longitudinal_offsets_m.begin(),
                             output.longitudinal_offsets_m.end()));
  EXPECT_TRUE(planner.selected().has_value());
  EXPECT_EQ(output.output_horizon_diagnostic.failure,
            sl::OutputHorizonFailure::NONE);
}

TEST(StaticInputs, FinalVer3ReferenceAndMapShareCoordinates) {
  const sl::PlannerConfig config;
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  EXPECT_EQ(map.width(), 751U);
  EXPECT_EQ(map.height(), 759U);
  EXPECT_NEAR(map.resolution(), 0.1, 1.0e-12);
  for (const auto &point : frame.points()) {
    int x = 0, y = 0;
    EXPECT_TRUE(map.worldToCell(point.x, point.y, &x, &y));
  }
  int x = 0, y = 0;
  EXPECT_FALSE(map.worldToCell(map.originX() - 0.01, map.originY(), &x, &y));
  EXPECT_EQ(map.wallLevel(-1, 0), 9);
}

TEST(StaticInputs, OccupiedThresholdUsesRosMapOccupancySemantics) {
  const auto directory = std::filesystem::temp_directory_path() /
                         "state_lattice_overtake_planner_threshold_map";
  std::filesystem::create_directories(directory);
  const auto pgm = directory / "threshold.pgm";
  const auto yaml = directory / "threshold.yaml";
  {
    std::ofstream output(pgm, std::ios::binary);
    output << "P5\n5 2\n255\n";
    const std::vector<unsigned char> pixels{128U, 128U, 128U, 128U, 128U,
                                            0U,   0U,   0U,   0U,   0U};
    output.write(reinterpret_cast<const char *>(pixels.data()),
                 static_cast<std::streamsize>(pixels.size()));
  }
  {
    std::ofstream output(yaml);
    output << "image: threshold.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
           << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  }

  sl::GridMap map;
  sl::PlannerConfig config;
  std::string error;
  ASSERT_TRUE(map.load(yaml.string(), config, &error)) << error;
  // ROS map semantics classify mid-gray (occupancy ~= 0.50) as unknown/free
  // for collision purposes, while black (occupancy = 1.0) is occupied.
  EXPECT_EQ(map.occupiedCells().size(), 10U);
  EXPECT_EQ(
      std::count(map.occupiedCells().begin(), map.occupiedCells().end(), 1U),
      5);
  EXPECT_TRUE(map.occupied(0, 0));
  EXPECT_FALSE(map.occupied(0, 1));
}

TEST(LatticePlanner,
     GeneratesFifteenFivePointCandidatesForRuntimeLateralSetWithinDeadline) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 5.0;
  ego.curvature = 0.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  const auto start = std::chrono::steady_clock::now();
  const auto candidates = planner.generateCandidates(ego, {});
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();
  EXPECT_EQ(candidates.size(), 15U);
  EXPECT_LT(elapsed_ms, config.planning_deadline_ms);
  int feasible = 0;
  for (const auto &candidate : candidates) {
    EXPECT_EQ(candidate.representative.size(), 5U);
    if (candidate.feasible) {
      ++feasible;
    }
    for (std::size_t i = 1; i < candidate.dense.size(); ++i) {
      EXPECT_LE(std::hypot(candidate.dense[i].x - candidate.dense[i - 1U].x,
                           candidate.dense[i].y - candidate.dense[i - 1U].y),
                config.collision_max_step_m + 1.0e-3);
      EXPECT_LE(std::abs(sl::normalizeAngle(candidate.dense[i].yaw -
                                            candidate.dense[i - 1U].yaw)),
                config.collision_max_yaw_step_rad + 1.0e-3);
    }
  }
  EXPECT_GT(feasible, 0);
}

TEST(LatticePlanner, RearSamplingAngleControlsSideSpreadWithoutReverseCommand) {
  sl::PlannerConfig narrow_config;
  narrow_config.rear_sampling_angle_rad = 0.1;
  auto frame = sampledSquareFrame();
  auto map = openGrid(narrow_config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, narrow_config));
  sl::EgoState ego;
  ego.x = 10.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::LatticePlanner narrow(narrow_config, &frame, &map);
  const auto narrow_path = narrow.generateRearSafetyPath(ego, 0.4);

  sl::PlannerConfig wide_config = narrow_config;
  wide_config.rear_sampling_angle_rad = 0.5;
  sl::LatticePlanner wide(wide_config, &frame, &map);
  const auto wide_path = wide.generateRearSafetyPath(ego, 0.4);
  ASSERT_EQ(narrow_path.size(), 5U);
  ASSERT_EQ(wide_path.size(), 5U);
  EXPECT_GT(std::abs(wide_path.back().d), std::abs(narrow_path.back().d));
  EXPECT_LT(wide_path.back().s, ego.frenet.s);
  EXPECT_NEAR(wide_path.front().time_sec,
              wide_config.rear_prediction_horizon_sec, 1.0e-12);
  EXPECT_NEAR(wide_path.back().time_sec, 0.0, 1.0e-12);
  for (std::size_t i = 1; i < wide_path.size(); ++i) {
    EXPECT_LT(wide_path[i].time_sec, wide_path[i - 1U].time_sec);
  }
}

TEST(LatticePlanner, HighSpeedPreparesAndLowSpeedExecutesSameSafePass) {
  sl::PlannerConfig config;
  sl::PlannerConfig pp_aligned_config = config;
  pp_aligned_config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  sl::applyResolvedControllerTrackabilityEnvelope(&pp_aligned_config, 0.5236,
                                                  0.35);
  pp_aligned_config.exact_cartesian_execution_enabled = true;
  config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::INSTANT;
  sl::applyResolvedControllerTrackabilityEnvelope(&config, 0.5236, 0.35);
  ASSERT_DOUBLE_EQ(config.planner_max_steer_rad, 0.5236);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.35);
  ASSERT_DOUBLE_EQ(pp_aligned_config.planner_max_steer_rad, 0.64);
  ASSERT_DOUBLE_EQ(pp_aligned_config.max_steer_rate_radps, 128.0);
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.x = 11.0;
  opponent.y = 0.0;
  opponent.yaw = 0.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;

  sl::EgoState fast;
  fast.x = 2.0;
  fast.y = 0.0;
  fast.yaw = 0.0;
  fast.speed_mps = 5.0;
  fast.frenet = frame.project(fast.x, fast.y, fast.yaw);
  fast.valid = true;
  sl::LatticePlanner fast_planner(config, &frame, &map);
  sl::LatticePlanner fast_baseline(config, &frame, &map);
  sl::TrialSpeedEvidence prepare_evidence{};
  const auto prepare =
      fast_planner.update(fast, {opponent}, true, 1.0, {}, &prepare_evidence);
  const auto prepare_baseline =
      fast_baseline.update(fast, {opponent}, true, 1.0);
  EXPECT_TRUE(prepare.mode == sl::BehaviorMode::PREPARE_OVERTAKE_LEFT ||
              prepare.mode == sl::BehaviorMode::PREPARE_OVERTAKE_RIGHT)
      << prepare.reason << " / "
      << sl::toString(prepare.output_horizon_diagnostic.failure);
  EXPECT_FALSE(prepare.safe_lateral);
  EXPECT_LE(prepare.speed_cap_mps, prepare.candidate_speed_limit_mps + 1.0e-9);
  EXPECT_EQ(prepare_evidence.capture_state,
            sl::SpeedEvidenceCaptureState::TRIAL_COMPLETE_UNSEALED);
  EXPECT_EQ(prepare_evidence.branch,
            sl::SpeedTransitionBranch::PREPARE_OVERTAKE_DIRECT);
  EXPECT_EQ(prepare_evidence.state_changed, 1U);
  EXPECT_EQ(prepare.mode, prepare_baseline.mode);
  EXPECT_EQ(prepare.reason, prepare_baseline.reason);
  EXPECT_DOUBLE_EQ(prepare.speed_cap_mps, prepare_baseline.speed_cap_mps);
  EXPECT_EQ(prepare.safe_lateral, prepare_baseline.safe_lateral);
  EXPECT_EQ(prepare.emergency_stop, prepare_baseline.emergency_stop);

  sl::LatticePlanner pp_aligned_planner(pp_aligned_config, &frame, &map);
  const auto pp_aligned_pass =
      pp_aligned_planner.update(fast, {opponent}, true, 1.0);
  EXPECT_TRUE(pp_aligned_pass.mode == sl::BehaviorMode::OVERTAKE_LEFT ||
              pp_aligned_pass.mode == sl::BehaviorMode::OVERTAKE_RIGHT)
      << pp_aligned_pass.reason << " / "
      << sl::toString(pp_aligned_pass.output_horizon_diagnostic.failure);
  EXPECT_TRUE(pp_aligned_pass.safe_lateral);
  EXPECT_EQ(pp_aligned_pass.execution_geometry_kind,
            sl::ExecutionGeometryKind::EXACT_CARTESIAN);
  EXPECT_EQ(pp_aligned_pass.lateral_offsets_m,
            std::vector<double>({0.0, 0.0}));
  EXPECT_GT(pp_aligned_pass.feasible_candidates, 0);
  EXPECT_EQ(pp_aligned_pass.output_horizon_diagnostic.failure,
            sl::OutputHorizonFailure::NONE);

  sl::EgoState slow = fast;
  slow.speed_mps = config.safe_stop_speed_mps;
  sl::LatticePlanner slow_planner(config, &frame, &map);
  sl::TrialSpeedEvidence pass_evidence{};
  const auto pass =
      slow_planner.update(slow, {opponent}, true, 1.0, {}, &pass_evidence);
  EXPECT_TRUE(pass.mode == sl::BehaviorMode::OVERTAKE_LEFT ||
              pass.mode == sl::BehaviorMode::OVERTAKE_RIGHT)
      << pass.reason << " / "
      << sl::toString(pass.output_horizon_diagnostic.failure)
      << " shift=" << pass.output_horizon_diagnostic.nearest_shift
      << " layout=" << pass.output_horizon_diagnostic.layout_offset
      << " waypoint=" << pass.output_horizon_diagnostic.waypoint_index
      << " piece=" << pass.output_horizon_diagnostic.interpolation_piece
      << " s=" << pass.output_horizon_diagnostic.s_m
      << " d=" << pass.output_horizon_diagnostic.d_m;
  EXPECT_TRUE(pass.safe_lateral);
  EXPECT_EQ(pass_evidence.capture_state,
            sl::SpeedEvidenceCaptureState::TRIAL_COMPLETE_UNSEALED);
  EXPECT_EQ(pass_evidence.branch,
            sl::SpeedTransitionBranch::CANDIDATE_JERK_LIMITED);
  EXPECT_EQ(pass.lateral_offsets_m.size(), 20U);

  const auto &true_diagnostic =
      slow_planner.lastPlanningCycleMetrics().pass_clearance_diagnostic;
  EXPECT_TRUE(true_diagnostic.evaluated);
  EXPECT_TRUE(true_diagnostic.inputs_valid);
  EXPECT_TRUE(true_diagnostic.observed_predicate);
  EXPECT_EQ(std::string(true_diagnostic.target_id.data()), opponent.id);
  EXPECT_TRUE(true_diagnostic.side == -1 || true_diagnostic.side == 1);
  EXPECT_GE(true_diagnostic.actual_separation_m + 1.0e-6,
            true_diagnostic.required_separation_m);
  EXPECT_GE(true_diagnostic.margin_m, -1.0e-6);

  slow_planner.resetManeuverState();
  const auto reset = slow_planner.update(slow, {}, true, 1.1);
  EXPECT_EQ(reset.mode, sl::BehaviorMode::FREE_RUN);
  const auto &reset_diagnostic =
      slow_planner.lastPlanningCycleMetrics().pass_clearance_diagnostic;
  EXPECT_FALSE(reset_diagnostic.evaluated);
  EXPECT_FALSE(reset_diagnostic.observed_predicate);
  EXPECT_TRUE(std::string(reset_diagnostic.target_id.data()).empty());
}

TEST(LatticePlanner,
     PassClearanceDiagnosticCapturesFalsePostFeasibilityFilter) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  const auto ego = sideRoleEgo(frame, 2.0, 0.0, 2.0, 5.0);
  auto target = sideRoleOpponent(frame, "d2", 11.0, 0.0, 0.0, 5.0);
  target.uncertainty_y_m = 10.0;

  sl::LatticePlanner planner(config, &frame, &map);
  const auto output = planner.update(ego, {target}, true, 5.0);
  EXPECT_EQ(output.reason, "follow_blocked_no_pass_clearance");
  const auto &diagnostic =
      planner.lastPlanningCycleMetrics().pass_clearance_diagnostic;
  EXPECT_TRUE(diagnostic.evaluated);
  EXPECT_TRUE(diagnostic.inputs_valid);
  EXPECT_FALSE(diagnostic.observed_predicate);
  EXPECT_EQ(std::string(diagnostic.target_id.data()), target.id);
  EXPECT_DOUBLE_EQ(diagnostic.target_d_m, target.frenet.d);
  EXPECT_DOUBLE_EQ(diagnostic.target_uncertainty_y_m, target.uncertainty_y_m);
  EXPECT_DOUBLE_EQ(diagnostic.target_observation_stamp_sec, target.stamp_sec);
  EXPECT_TRUE(diagnostic.side == -1 || diagnostic.side == 1);
  EXPECT_LT(diagnostic.actual_separation_m, diagnostic.required_separation_m);
  EXPECT_LT(diagnostic.margin_m, -1.0e-6);
}

TEST(LatticePlanner,
     PassContinuationUsesOnlyPriorLiveOvertakeAndClearsWithoutResurrection) {
  sl::PlannerConfig config;
  config.front_detection_radius_m = 20.0;
  config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  sl::applyResolvedControllerTrackabilityEnvelope(&config, 0.5236, 0.35);
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 5.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::OpponentState target;
  target.id = "d2";
  target.x = 18.0;
  target.y = 0.0;
  target.yaw = 0.0;
  target.stamp_sec = 1.0;
  target.frenet = frame.project(target.x, target.y, target.yaw);
  target.valid = true;

  sl::LatticePlanner planner(config, &frame, &map);
  const auto first = planner.update(ego, {target}, true, 1.0);
  ASSERT_TRUE(first.mode == sl::BehaviorMode::OVERTAKE_LEFT ||
              first.mode == sl::BehaviorMode::OVERTAKE_RIGHT)
      << first.reason;
  EXPECT_TRUE(first.pass_continuation_latched);
  EXPECT_FALSE(first.pass_continuation_active);
  EXPECT_EQ(first.pass_continuation_target_id, target.id);
  EXPECT_GE(
      first.pass_continuation_required_arc_m,
      sl::requiredLateralTransitionDistance(config, ego.speed_mps, ego.frenet.d,
                                            first.pass_continuation_goal_d_m) -
          config.tie_break_epsilon);

  target.stamp_sec = 1.1;
  const auto continued = planner.update(ego, {target}, true, 1.1);
  ASSERT_TRUE(continued.mode == sl::BehaviorMode::OVERTAKE_LEFT ||
              continued.mode == sl::BehaviorMode::OVERTAKE_RIGHT)
      << continued.reason;
  EXPECT_TRUE(continued.pass_continuation_latched);
  EXPECT_TRUE(continued.pass_continuation_active);

  const auto stale = planner.update(ego, {target}, false, 1.2);
  EXPECT_EQ(stale.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_FALSE(stale.pass_continuation_latched);
  EXPECT_FALSE(stale.pass_continuation_active);

  target.stamp_sec = 1.3;
  const auto after_stale = planner.update(ego, {target}, true, 1.3);
  EXPECT_FALSE(after_stale.pass_continuation_active);

  const auto target_missing = planner.update(ego, {}, true, 1.4);
  EXPECT_FALSE(target_missing.pass_continuation_latched);
  EXPECT_FALSE(target_missing.pass_continuation_active);
}

TEST(LatticePlanner,
     MovingTargetCurrentDFollowRequiresStoppedTargetSafetyProof) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0};
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 3.0;
  ego.yaw = 0.0;
  ego.speed_mps = 0.5;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  sl::OpponentState moving;
  moving.id = "d2";
  moving.x = 5.0;
  moving.y = 0.0;
  moving.yaw = 0.0;
  moving.speed_mps = 2.0;
  moving.vx_mps = 2.0;
  moving.frenet = frame.project(moving.x, moving.y, moving.yaw);
  moving.valid = moving.frenet.valid;
  ASSERT_TRUE(moving.valid);

  sl::LatticePlanner moving_planner(config, &frame, &map);
  const auto follow = moving_planner.update(ego, {moving}, true, 1.0);
  ASSERT_EQ(follow.mode, sl::BehaviorMode::FOLLOW_BLOCKED) << follow.reason;
  EXPECT_EQ(follow.reason, "moving_target_current_d_follow");
  EXPECT_TRUE(follow.safe_lateral);
  EXPECT_FALSE(follow.emergency_stop);
  EXPECT_EQ(follow.intent, sl::SolverHorizonIntent::MANEUVER_AUTHORIZED);
  EXPECT_EQ(follow.lateral_offsets_m.size(),
            static_cast<std::size_t>(config.horizon_points));
  EXPECT_EQ(follow.lateral_offsets_m.size(), follow.speed_caps_mps.size());
  EXPECT_EQ(follow.lateral_offsets_m.size(),
            follow.longitudinal_offsets_m.size());
  ASSERT_TRUE(moving_planner.selected().has_value());
  EXPECT_NEAR(moving_planner.selected()->goal_d_m, ego.frenet.d, 1.0e-6);
  EXPECT_GT(moving_planner.selected()->required_arc_m,
            config.minimum_obstacle_transition_distance_m);

  moving.speed_mps = 0.0;
  moving.vx_mps = 0.0;
  sl::LatticePlanner stopped_planner(config, &frame, &map);
  const auto stopped = stopped_planner.update(ego, {moving}, true, 1.0);
  EXPECT_NE(stopped.reason, "moving_target_current_d_follow");
  EXPECT_FALSE(stopped.safe_lateral);
}

TEST(LatticePlanner, MovingTargetFollowKeepsCurrentDWithoutStageClearance) {
  sl::PlannerConfig config;
  // Keep this state-arbitration fixture independent from the production
  // trackability-limit change.
  config.hard_max_steer_rad = 0.5585;
  config.planner_max_steer_rad = 0.5236;
  config.max_steer_rate_radps = 0.35;
  config.lateral_targets_m = {0.0, 1.99};
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  config.opponent_lateral_tracking_margin_m = 0.0;
  config.opponent_hard_clearance_m = 0.0;
  config.overtake_permission_profile_enabled = true;
  config.default_overtake_allowed = false;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 3.0;
  ego.yaw = 0.0;
  ego.speed_mps = 0.5;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  sl::OpponentState moving;
  moving.id = "d2";
  moving.x = 5.0;
  moving.y = -2.0;
  moving.yaw = 0.0;
  moving.speed_mps = 2.0;
  moving.vx_mps = 2.0;
  moving.frenet = frame.project(moving.x, moving.y, moving.yaw);
  moving.valid = moving.frenet.valid;
  ASSERT_TRUE(moving.valid);

  sl::LatticePlanner planner(config, &frame, &map);
  const auto follow = planner.update(ego, {moving}, true, 1.0);
  ASSERT_EQ(follow.mode, sl::BehaviorMode::FOLLOW_BLOCKED) << follow.reason;
  EXPECT_EQ(follow.reason, "moving_target_current_d_follow");
  EXPECT_TRUE(follow.safe_lateral);
  ASSERT_TRUE(planner.selected().has_value());
  EXPECT_NEAR(planner.selected()->goal_d_m, ego.frenet.d, 1.0e-6);
}

TEST(LatticePlanner,
     MovingTargetFollowKeepsCurrentDUntilCenterwardStageHasFullClearance) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0};
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  config.opponent_lateral_tracking_margin_m = 0.0;
  config.opponent_hard_clearance_m = 0.0;
  config.overtake_permission_profile_enabled = true;
  config.default_overtake_allowed = false;
  auto frame = straightFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 1.99;
  ego.yaw = 0.0;
  ego.speed_mps = 0.5;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  sl::OpponentState moving;
  moving.id = "d2";
  moving.x = 7.0;
  moving.y = 0.0;
  moving.yaw = 0.0;
  moving.speed_mps = 2.0;
  moving.vx_mps = 2.0;
  moving.frenet = frame.project(moving.x, moving.y, moving.yaw);
  moving.valid = moving.frenet.valid;
  ASSERT_TRUE(moving.valid);

  sl::LatticePlanner planner(config, &frame, &map);
  const auto blocked = planner.update(ego, {moving}, true, 1.0);
  ASSERT_EQ(blocked.mode, sl::BehaviorMode::FOLLOW_BLOCKED) << blocked.reason;
  EXPECT_EQ(blocked.reason, "moving_target_current_d_follow");
  ASSERT_TRUE(planner.selected().has_value());
  EXPECT_NEAR(planner.selected()->goal_d_m, ego.frenet.d, 1.0e-6);
  EXPECT_GT(planner.selected()->required_arc_m,
            config.minimum_obstacle_transition_distance_m);

  moving.x = 25.0;
  moving.frenet = frame.project(moving.x, moving.y, moving.yaw);
  const auto clear = planner.update(ego, {moving}, true, 1.1);
  ASSERT_EQ(clear.mode, sl::BehaviorMode::FOLLOW_BLOCKED) << clear.reason;
  EXPECT_EQ(clear.reason, "moving_target_staged_lateral_return");
  ASSERT_TRUE(planner.selected().has_value());
  EXPECT_LT(std::abs(planner.selected()->goal_d_m), std::abs(ego.frenet.d));
}

TEST(LatticePlanner, Dev3D1MovingReleaseFixtureRemainsFailClosedAtWall) {
  sl::PlannerConfig config;
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  const auto &ego_source = sl::test_fixture::kD1MovingReleaseEgo;
  sl::EgoState ego;
  ego.x = ego_source.x_m;
  ego.y = ego_source.y_m;
  ego.yaw = ego_source.yaw_rad;
  ego.speed_mps = ego_source.speed_mps;
  ego.stamp_sec = 100.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  std::vector<sl::OpponentState> opponents;
  for (const auto &source : sl::test_fixture::kD1MovingReleaseOpponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.yaw = source.yaw_rad;
    opponent.speed_mps = source.speed_mps;
    opponent.vx_mps = std::cos(opponent.yaw) * opponent.speed_mps;
    opponent.vy_mps = std::sin(opponent.yaw) * opponent.speed_mps;
    opponent.uncertainty_x_m = 0.15;
    opponent.uncertainty_y_m = 0.15;
    opponent.stamp_sec = 100.0;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.valid = opponent.frenet.valid;
    ASSERT_TRUE(opponent.valid);
    opponents.push_back(opponent);
  }

  sl::LatticePlanner planner(config, &frame, &map);
  const auto output = planner.update(ego, opponents, true, 100.0);
  ASSERT_EQ(output.mode, sl::BehaviorMode::FOLLOW_BLOCKED) << output.reason;
  EXPECT_EQ(output.reason, "follow_blocked_no_pass_clearance");
  EXPECT_FALSE(planner.selected().has_value());
  EXPECT_GT(output.speed_cap_mps, config.safe_stop_speed_mps);
}

TEST(LatticePlanner, Dev3D2ThreeFeasibleCandidatesReproduceCostSafeStop) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};
  config.max_adaptive_lateral_offset_m = 2.2;
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  const auto &ego_source = sl::test_fixture::kD2CostStopEgo;
  sl::EgoState ego;
  ego.x = ego_source.x_m;
  ego.y = ego_source.y_m;
  ego.yaw = ego_source.yaw_rad;
  ego.speed_mps = ego_source.speed_mps;
  ego.stamp_sec = 100.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  std::vector<sl::OpponentState> opponents;
  for (const auto &source : sl::test_fixture::kD2CostStopOpponents) {
    sl::OpponentState opponent;
    opponent.id = std::string(source.vehicle_id);
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.yaw = source.yaw_rad;
    opponent.speed_mps = source.speed_mps;
    opponent.vx_mps = std::cos(opponent.yaw) * opponent.speed_mps;
    opponent.vy_mps = std::sin(opponent.yaw) * opponent.speed_mps;
    opponent.uncertainty_x_m = 0.15;
    opponent.uncertainty_y_m = 0.15;
    opponent.stamp_sec = 99.934344;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.valid = opponent.frenet.valid;
    ASSERT_TRUE(opponent.valid);
    opponents.push_back(opponent);
  }

  sl::LatticePlanner planner(config, &frame, &map);
  auto target_only = opponents.back();
  target_only.stamp_sec = 99.9;
  const auto seed = planner.update(ego, {target_only}, true, 99.9);
  EXPECT_EQ(seed.target_id, "d3");
  const auto output = planner.update(ego, opponents, true, 100.0);
  EXPECT_EQ(output.generated_candidates, 15);
  EXPECT_GE(output.feasible_candidates, 3);
  const auto observed_pass_band =
      std::count_if(planner.candidates().cbegin(), planner.candidates().cend(),
                    [](const auto &candidate) {
                      return candidate.feasible &&
                             std::abs(candidate.goal_d_m - (-1.0)) <= 1.0e-9;
                    });
  EXPECT_EQ(observed_pass_band, 3);
  EXPECT_GE(output.rejected_wall_candidates, 5);
  EXPECT_EQ(output.rejected_opponent_candidates, 6);
  EXPECT_EQ(output.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(output.reason, "cost_stop_threshold");
  EXPECT_GE(output.minimum_cost, config.stop_cost);
  EXPECT_DOUBLE_EQ(output.speed_cap_mps, config.safe_stop_speed_mps);
  EXPECT_TRUE(std::all_of(
      output.speed_caps_mps.begin(), output.speed_caps_mps.end(),
      [&](double cap_mps) { return cap_mps == config.safe_stop_speed_mps; }));

  auto cleared_opponents = opponents;
  for (std::size_t i = 0; i < cleared_opponents.size(); ++i) {
    auto &opponent = cleared_opponents[i];
    const double clear_s = ego.frenet.s + 20.0 + 5.0 * i;
    const auto clear_pose = frame.frenetToCartesian(clear_s, opponent.frenet.d);
    opponent.x = clear_pose.x;
    opponent.y = clear_pose.y;
    opponent.yaw = frame.interpolate(clear_s).yaw;
    opponent.vx_mps = std::cos(opponent.yaw) * opponent.speed_mps;
    opponent.vy_mps = std::sin(opponent.yaw) * opponent.speed_mps;
    opponent.stamp_sec = 100.1;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    ASSERT_TRUE(opponent.frenet.valid);
  }
  const auto released = planner.update(ego, cleared_opponents, true, 100.1);
  EXPECT_NE(released.mode, sl::BehaviorMode::SAFE_STOP) << released.reason;
  EXPECT_NE(released.reason, "cost_stop_latched");
  double released_speed_cap_mps = released.speed_cap_mps;
  for (int cycle = 0; cycle < 8 && released_speed_cap_mps <=
                                       config.safe_stop_speed_mps + 1.0e-9;
       ++cycle) {
    for (auto &opponent : cleared_opponents) {
      opponent.stamp_sec = 100.2 + 0.05 * cycle;
    }
    const auto advancing =
        planner.update(ego, cleared_opponents, true, 100.2 + 0.05 * cycle);
    EXPECT_NE(advancing.mode, sl::BehaviorMode::SAFE_STOP) << advancing.reason;
    released_speed_cap_mps = advancing.speed_cap_mps;
  }
  EXPECT_GT(released_speed_cap_mps, config.safe_stop_speed_mps);
}

TEST(LatticePlanner, PermissionLookaheadBlocksOnlyNewPassStart) {
  sl::PlannerConfig config;
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 8.0;
  config.overtake_permission_rules.push_back({"deny_ahead", 15.0, 25.0, false});
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  sl::EgoState ego;
  ego.x = 10.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 0.2;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.x = 19.0;
  opponent.y = 0.0;
  opponent.yaw = 0.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;
  const auto output = planner.update(ego, {opponent}, true, 1.0);
  EXPECT_EQ(output.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_FALSE(output.overtake_permission_allowed);
  EXPECT_EQ(output.overtake_permission_section_name, "deny_ahead");
  EXPECT_EQ(output.overtake_permission_reason, "lookahead_section_disallowed");
}

TEST(LatticePlanner, FollowBlockedRegulatesClearanceAndClosingSpeed) {
  sl::PlannerConfig config;
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules.push_back(
      {"deny_follow", 0.0, 119.0, false});
  config.follow_desired_extra_gap_m = 1.0;
  config.follow_gap_gain_per_s = 0.8;
  config.follow_closing_speed_gain = 0.5;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  sl::EgoState ego;
  ego.x = 10.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 3.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;

  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.stamp_sec = 1.0;
  opponent.y = 0.0;
  opponent.yaw = 0.0;
  opponent.speed_mps = 3.0;
  opponent.vx_mps = 3.0;
  opponent.valid = true;

  const double physical_buffer =
      config.wheel_base_m + config.front_overhang_m +
      config.opponent_longitudinal_tracking_margin_m + config.rear_overhang_m +
      opponent.uncertainty_x_m + config.opponent_hard_clearance_m +
      config.pass_lateral_extra_margin_m;
  const auto evaluate_at_gap = [&](double gap_m, double ego_speed_mps) {
    sl::LatticePlanner planner(config, &frame, &map);
    sl::EgoState evaluated_ego = ego;
    evaluated_ego.speed_mps = ego_speed_mps;
    opponent.x = evaluated_ego.x + gap_m;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    return planner.update(evaluated_ego, {opponent}, true, 1.0);
  };

  const double desired_gap =
      physical_buffer + config.follow_desired_extra_gap_m;
  const auto at_target = evaluate_at_gap(desired_gap, opponent.speed_mps);
  ASSERT_EQ(at_target.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_NEAR(at_target.speed_cap_mps, opponent.speed_mps, 1.0e-6);

  const auto too_far = evaluate_at_gap(desired_gap + 1.0, opponent.speed_mps);
  ASSERT_EQ(too_far.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_GT(too_far.speed_cap_mps, at_target.speed_cap_mps);

  const auto too_close = evaluate_at_gap(desired_gap - 0.5, opponent.speed_mps);
  ASSERT_EQ(too_close.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_LT(too_close.speed_cap_mps, at_target.speed_cap_mps);

  const auto closing = evaluate_at_gap(desired_gap, opponent.speed_mps + 2.0);
  ASSERT_EQ(closing.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_LT(closing.speed_cap_mps, at_target.speed_cap_mps);

  opponent.speed_mps = 0.0;
  opponent.vx_mps = 0.0;
  const auto stopped =
      evaluate_at_gap(desired_gap - 0.5, config.safe_stop_speed_mps);
  ASSERT_EQ(stopped.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_LE(stopped.speed_cap_mps, config.safe_stop_speed_mps + 1.0e-9);

  const auto high_speed_closing = evaluate_at_gap(desired_gap + 3.0, 8.0);
  ASSERT_EQ(high_speed_closing.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_LE(high_speed_closing.speed_cap_mps,
            config.safe_stop_speed_mps + 1.0e-9);
}

TEST(LatticePlanner, MissingTargetPredictsThenRecoversAndResets) {
  sl::PlannerConfig config;
  config.target_missing_prediction_grace_sec = 0.2;
  config.target_missing_forget_sec = 0.6;
  config.front_release_cycles = 2;
  config.return_required_cycles = 3;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 0.2;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.x = 11.0;
  opponent.y = 0.0;
  opponent.yaw = 0.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;
  auto output = planner.update(ego, {opponent}, true, 0.0);
  ASSERT_TRUE(planner.active());
  bool saw_recovery = false;
  for (int cycle = 1; cycle <= 30 && planner.active(); ++cycle) {
    output = planner.update(ego, {}, true, 0.1 * cycle);
    saw_recovery = saw_recovery ||
                   output.mode == sl::BehaviorMode::YIELD_BEHIND ||
                   output.mode == sl::BehaviorMode::ABORT_RECOVERY;
  }
  EXPECT_TRUE(saw_recovery);
  EXPECT_FALSE(planner.active());
  EXPECT_EQ(output.reason, "target_missing_recovery_complete");
  EXPECT_TRUE(planner.targetId().empty());
  EXPECT_TRUE(planner.candidates().empty());
  EXPECT_FALSE(planner.selected().has_value());
}

TEST(LatticePlanner, MpcHealthGuardUsesReleaseHysteresis) {
  sl::PlannerConfig config;
  config.mpc_health_release_samples = 3;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 2.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 2;
  health.solve_time_ms = 5.0;
  health.age_sec = 0.0;
  health.sample_sequence = 1U;
  auto output = planner.update(ego, {}, true, 0.0, health);
  EXPECT_EQ(output.mode, sl::BehaviorMode::SPEED_GUARD);
  health.infeasible_count = 0;
  for (std::uint64_t sequence = 2U; sequence <= 4U; ++sequence) {
    health.sample_sequence = sequence;
    output = planner.update(ego, {}, true, 0.1 * sequence, health);
  }
  EXPECT_EQ(output.mode, sl::BehaviorMode::FREE_RUN);
  EXPECT_FALSE(output.active);
}

TEST(LatticePlanner, DetectsOvertakesAndReturnsOnlyAfterRearAuditHysteresis) {
  sl::PlannerConfig config;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 0.2;
  ego.curvature = 0.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.x = 11.0;
  opponent.y = 0.0;
  opponent.yaw = 0.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;
  auto output = planner.update(ego, {opponent}, true, 1.0);
  EXPECT_TRUE(planner.active());
  EXPECT_TRUE(output.active);
  EXPECT_NE(output.mode, sl::BehaviorMode::FREE_RUN);

  ego.x = 20.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  opponent.x = 10.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  for (int cycle = 0; cycle < 12 && planner.active(); ++cycle) {
    output = planner.update(ego, {opponent}, true, 2.0 + cycle * 0.05);
  }
  EXPECT_FALSE(planner.active());
  EXPECT_FALSE(output.active);
  EXPECT_EQ(output.reason, "return_complete");
}

TEST(LatticePlanner, CostThresholdProducesSafeStopV3WhenLateralPathIsSafe) {
  sl::PlannerConfig config;
  config.free_run_return_cost = 20;
  config.stop_cost = 29;
  config.safe_stop_release_cost = 28;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 5.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.x = 9.0;
  opponent.y = -2.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;
  sl::TrialSpeedEvidence evidence{};
  const auto output = planner.update(ego, {opponent}, true, 1.0, {}, &evidence);
  EXPECT_EQ(output.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_TRUE(output.safe_lateral) << output.reason;
  EXPECT_EQ(evidence.capture_state,
            sl::SpeedEvidenceCaptureState::TRIAL_COMPLETE_UNSEALED);
  EXPECT_EQ(evidence.branch, sl::SpeedTransitionBranch::CANDIDATE_LOGICAL_STOP);
  EXPECT_EQ(output.lateral_offsets_m.size(), 20U);
  EXPECT_TRUE(std::all_of(
      output.speed_caps_mps.begin(), output.speed_caps_mps.end(),
      [&config](double speed) {
        return std::abs(speed - config.safe_stop_speed_mps) < 1.0e-9;
      }));
}

TEST(LatticePlanner, NoFeasibleCandidateProducesSpeedOnlySafeStopState) {
  const sl::PlannerConfig config;
  auto frame = sampledSquareFrame();
  auto map = testGrid(config, true);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 5.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.x = 9.0;
  opponent.y = 0.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;
  const auto output = planner.update(ego, {opponent}, true, 1.0);
  EXPECT_EQ(output.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_FALSE(output.safe_lateral);
  EXPECT_TRUE(output.lateral_offsets_m.empty());
  EXPECT_EQ(output.reason, "current_pose_hard_collision");
  const auto payload = sl::makeWirePayload(output, 1U);
  EXPECT_EQ(payload.kind, sl::WireKind::SPEED_ONLY_V2);
}

TEST(LatticePlanner, CurrentPoseRearOnlyNonClosingOpponentIsExemptInFreeRun) {
  const sl::PlannerConfig config;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  sl::EgoState ego;
  ego.x = 10.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 2.0;
  ego.stamp_sec = 1.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  sl::OpponentState opponent;
  opponent.id = "d2";
  // The inflated opponent front is 0.10 m behind the inflated ego rear, so
  // this remains a hard-clearance violation while being geometrically
  // rear-only.
  opponent.x = 7.586;
  opponent.y = 0.0;
  opponent.yaw = 0.0;
  opponent.speed_mps = 0.0;
  opponent.vx_mps = 0.0;
  opponent.vy_mps = 0.0;
  opponent.stamp_sec = 1.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;

  const auto output = planner.update(ego, {opponent}, true, 1.0);
  EXPECT_NE(output.reason, "current_pose_hard_collision");
  EXPECT_EQ(output.current_opponent_relation,
            sl::CurrentPoseOpponentRelation::REAR_ONLY);
  EXPECT_TRUE(output.current_pose_rear_only_exempt);
  EXPECT_EQ(output.current_pose_rear_only_exempt_opponent_id, "d2");
  EXPECT_LT(output.current_pose_rear_only_exempt_clearance_m,
            config.opponent_hard_clearance_m);
}

TEST(LatticePlanner, CurrentPoseFrontSideAndClosingRearRemainFailClosed) {
  const sl::PlannerConfig config;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  const auto make_ego = [&frame]() {
    sl::EgoState ego;
    ego.x = 10.0;
    ego.y = 0.0;
    ego.yaw = 0.0;
    ego.speed_mps = 2.0;
    ego.stamp_sec = 1.0;
    ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
    ego.valid = true;
    return ego;
  };
  const auto make_opponent = [&frame](double x, double y, double vx_mps) {
    sl::OpponentState opponent;
    opponent.id = "d2";
    opponent.x = x;
    opponent.y = y;
    opponent.yaw = 0.0;
    opponent.vx_mps = vx_mps;
    opponent.stamp_sec = 1.0;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.valid = true;
    return opponent;
  };
  const auto expect_stop = [&config, &frame, &map, &make_ego](
                               const sl::OpponentState &opponent,
                               sl::CurrentPoseOpponentRelation relation) {
    sl::LatticePlanner planner(config, &frame, &map);
    const auto output = planner.update(make_ego(), {opponent}, true, 1.0);
    EXPECT_EQ(output.reason, "current_pose_hard_collision");
    EXPECT_EQ(output.current_collision_kind, sl::CollisionKind::OPPONENT);
    EXPECT_EQ(output.current_opponent_relation, relation);
    EXPECT_FALSE(output.current_pose_rear_only_exempt);
  };

  expect_stop(make_opponent(12.414, 0.0, 0.0),
              sl::CurrentPoseOpponentRelation::FRONT_OR_OVERLAP);
  expect_stop(make_opponent(10.0, 1.50, 0.0),
              sl::CurrentPoseOpponentRelation::SIDE_OVERLAP);
  expect_stop(make_opponent(7.586, 0.0, 2.1),
              sl::CurrentPoseOpponentRelation::REAR_ONLY);

  auto stale_rear = make_opponent(7.586, 0.0, 0.0);
  stale_rear.stamp_sec = 0.4;
  expect_stop(stale_rear, sl::CurrentPoseOpponentRelation::REAR_ONLY);
  auto future_rear = make_opponent(7.586, 0.0, 0.0);
  future_rear.stamp_sec = 1.1;
  expect_stop(future_rear, sl::CurrentPoseOpponentRelation::REAR_ONLY);
}

TEST(LatticePlanner, RearOnlyExemptionDoesNotLeakIntoNextClosingCycle) {
  const sl::PlannerConfig config;
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  const auto make_ego = [&frame](double stamp_sec) {
    sl::EgoState ego;
    ego.x = 10.0;
    ego.y = 0.0;
    ego.yaw = 0.0;
    ego.speed_mps = 2.0;
    ego.stamp_sec = stamp_sec;
    ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
    ego.valid = true;
    return ego;
  };
  const auto make_rear = [&frame](double stamp_sec, double vx_mps) {
    sl::OpponentState rear;
    rear.id = "rear";
    rear.x = 7.586;
    rear.y = 0.0;
    rear.yaw = 0.0;
    rear.vx_mps = vx_mps;
    rear.stamp_sec = stamp_sec;
    rear.frenet = frame.project(rear.x, rear.y, rear.yaw);
    rear.valid = true;
    return rear;
  };

  const auto exempt =
      planner.update(make_ego(1.0), {make_rear(1.0, 0.0)}, true, 1.0);
  ASSERT_TRUE(exempt.current_pose_rear_only_exempt);
  ASSERT_NE(exempt.reason, "current_pose_hard_collision");

  const auto closing =
      planner.update(make_ego(2.0), {make_rear(2.0, 2.1)}, true, 2.0);
  EXPECT_EQ(closing.reason, "current_pose_hard_collision");
  EXPECT_EQ(closing.current_collision_kind, sl::CollisionKind::OPPONENT);
  EXPECT_EQ(closing.current_opponent_relation,
            sl::CurrentPoseOpponentRelation::REAR_ONLY);
  EXPECT_FALSE(closing.current_pose_rear_only_exempt);
}

TEST(LatticePlanner,
     RearOnlyExemptionFromFollowCannotStartNewPassOrCreateContinuationLatch) {
  sl::PlannerConfig config;
  config.front_detection_radius_m = 20.0;
  config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  sl::applyResolvedControllerTrackabilityEnvelope(&config, 0.5236, 0.35);
  config.overtake_permission_profile_enabled = true;
  config.default_overtake_allowed = true;
  config.front_detection_radius_m = 2.0;
  config.early_aware_enabled = true;
  config.early_aware_base_distance_m = 4.0;
  config.early_aware_min_distance_m = 8.0;
  config.early_aware_max_distance_m = 30.0;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules = {
      sl::OvertakePermissionRule{"initial_follow_only", 0.0, 3.0, false}};
  auto frame = sampledSquareFrame();
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner pass_control_planner(config, &frame, &map);
  sl::LatticePlanner rear_guarded_planner(config, &frame, &map);

  const auto make_ego = [&frame](double x) {
    sl::EgoState ego;
    ego.x = x;
    ego.y = 0.0;
    ego.yaw = 0.0;
    ego.speed_mps = 5.0;
    ego.stamp_sec = 1.0;
    ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
    ego.valid = true;
    return ego;
  };
  const auto make_opponent = [&frame](const std::string &id, double x,
                                      double vx_mps) {
    sl::OpponentState opponent;
    opponent.id = id;
    opponent.x = x;
    opponent.y = 0.0;
    opponent.yaw = 0.0;
    opponent.vx_mps = vx_mps;
    opponent.stamp_sec = 1.0;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.valid = true;
    return opponent;
  };

  // Establish the required prior FOLLOW_BLOCKED state in a no-pass section.
  const auto control_initial = pass_control_planner.update(
      make_ego(2.0), {make_opponent("front", 11.0, 0.0)}, true, 1.0);
  ASSERT_EQ(control_initial.mode, sl::BehaviorMode::FOLLOW_BLOCKED)
      << control_initial.reason;
  const auto guarded_initial = rear_guarded_planner.update(
      make_ego(2.0), {make_opponent("front", 11.0, 0.0)}, true, 1.0);
  ASSERT_EQ(guarded_initial.mode, sl::BehaviorMode::FOLLOW_BLOCKED)
      << guarded_initial.reason;

  // The paired control proves that the permitted straight-line scene has a
  // pass available when the rear-only hard-clearance vehicle is absent.
  const auto pass_control = pass_control_planner.update(
      make_ego(4.0), {make_opponent("front", 13.0, 0.0)}, true, 1.0);
  EXPECT_GT(pass_control.feasible_candidates, 0);
  EXPECT_TRUE(pass_control.mode == sl::BehaviorMode::OVERTAKE_LEFT ||
              pass_control.mode == sl::BehaviorMode::OVERTAKE_RIGHT)
      << pass_control.reason;

  // The matching cycle has a non-closing rear-only hard-clearance opponent.
  // Its future samples remain hard-checked, while its current pose cannot
  // authorize the otherwise available lateral maneuver.
  const auto output = rear_guarded_planner.update(
      make_ego(4.0),
      {make_opponent("front", 13.0, 0.0), make_opponent("rear", 1.586, 0.0)},
      true, 1.0);
  EXPECT_TRUE(output.current_pose_rear_only_exempt);
  EXPECT_EQ(output.current_pose_rear_only_exempt_opponent_id, "rear");
  EXPECT_EQ(output.feasible_candidates, 0);
  EXPECT_GT(output.rejected_opponent_candidates, 0);
  EXPECT_EQ(output.mode, sl::BehaviorMode::FOLLOW_BLOCKED) << output.reason;
  EXPECT_EQ(output.reason, "rear_only_exemption_follow_hold");
  EXPECT_FALSE(output.pass_continuation_latched);
  EXPECT_FALSE(output.pass_continuation_active);
  EXPECT_NE(output.mode, sl::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_NE(output.mode, sl::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_NE(output.mode, sl::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_NE(output.mode, sl::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_NE(output.mode, sl::BehaviorMode::SIDE_BY_SIDE_KEEP);
  EXPECT_NE(output.mode, sl::BehaviorMode::MERGE_BACK);
  EXPECT_NE(output.mode, sl::BehaviorMode::ABORT_RECOVERY);
  EXPECT_NE(output.mode, sl::BehaviorMode::YIELD_BEHIND);
}

TEST(LatticePlanner,
     PreventiveSideRoleElectsGeometryAndAppliesAsymmetricSpeedRequests) {
  auto frame = straightFrame();
  auto d1_config = sideRoleConfig("d1");
  auto map = openGrid(d1_config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, d1_config));
  auto d2_config = sideRoleConfig("d2");
  constexpr double now_sec = 1.0;
  constexpr double speed_mps = 3.0;

  const auto d1_ego = sideRoleEgo(frame, 10.0, 0.0, speed_mps, now_sec);
  auto d2_observed =
      sideRoleOpponent(frame, "d2", 10.5, 1.95, speed_mps, now_sec);
  d2_observed.vy_mps = -0.20;
  sl::LatticePlanner d1_planner(d1_config, &frame, &map);
  const auto d1_output =
      d1_planner.update(d1_ego, {d2_observed}, true, now_sec);
  EXPECT_TRUE(d1_output.preventive_side_role_active);
  EXPECT_FALSE(d1_output.preventive_side_role_leader);
  EXPECT_EQ(d1_output.preventive_side_role_peer_id, "d2");
  EXPECT_EQ(d1_output.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::FOLLOWER_YIELD);
  EXPECT_GT(d1_output.preventive_side_role_eligibility_diagnostic
                .cartesian_longitudinal_m,
            d1_config.preventive_side_role_tie_band_m);
  EXPECT_LE(d1_output.speed_cap_mps, speed_mps);
  EXPECT_FALSE(d1_output.pass_continuation_latched);

  const auto d2_ego = sideRoleEgo(frame, 10.5, 1.95, speed_mps, now_sec);
  auto d1_observed =
      sideRoleOpponent(frame, "d1", 10.0, 0.0, speed_mps, now_sec);
  d1_observed.vy_mps = 0.20;
  sl::LatticePlanner d2_planner(d2_config, &frame, &map);
  const auto d2_output =
      d2_planner.update(d2_ego, {d1_observed}, true, now_sec);
  EXPECT_TRUE(d2_output.preventive_side_role_active);
  EXPECT_TRUE(d2_output.preventive_side_role_leader);
  EXPECT_EQ(d2_output.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::LEADER_PROCEED);
  EXPECT_LT(d2_output.preventive_side_role_eligibility_diagnostic
                .cartesian_longitudinal_m,
            -d2_config.preventive_side_role_tie_band_m);
  EXPECT_LE(d2_output.speed_cap_mps, speed_mps);
  EXPECT_LT(d1_output.preventive_side_role_requested_speed_cap_mps,
            d2_output.preventive_side_role_requested_speed_cap_mps);

  auto reordered_config = sideRoleConfig("z9");
  sl::LatticePlanner reordered_planner(reordered_config, &frame, &map);
  d2_observed.id = "a0";
  const auto reordered =
      reordered_planner.update(d1_ego, {d2_observed}, true, now_sec);
  EXPECT_FALSE(reordered.preventive_side_role_leader);
  EXPECT_EQ(reordered.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::FOLLOWER_YIELD);
  EXPECT_NEAR(reordered.preventive_side_role_eligibility_diagnostic
                  .cartesian_longitudinal_m,
              d1_output.preventive_side_role_eligibility_diagnostic
                  .cartesian_longitudinal_m,
              1.0e-9);
}

TEST(LatticePlanner,
     PreventiveSideRoleFixedV2HeadingFixtureStillRequiresPredictiveRisk) {
  auto d1_config = sideRoleConfig("d1");
  auto d2_config = sideRoleConfig("d2");
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(
      frame.loadCsv(share + "/env/final_ver3/traj_mincurv_manual.csv", &error))
      << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       d1_config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, d1_config));
  constexpr double now_sec = 49.605;

  const auto make_ego = [&frame](double s_m, double d_m, double yaw_error_rad,
                                 double speed_mps) {
    const auto pose = frame.frenetToCartesian(s_m, d_m);
    sl::EgoState ego;
    ego.x = pose.x;
    ego.y = pose.y;
    ego.yaw = pose.yaw + yaw_error_rad;
    for (int i = 0; i < 10; ++i) {
      const auto location = frame.project(ego.x, ego.y, ego.yaw);
      ego.yaw = frame.interpolate(location.s).yaw + yaw_error_rad;
    }
    ego.curvature = pose.kappa;
    ego.speed_mps = speed_mps;
    ego.stamp_sec = now_sec;
    ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
    ego.valid = ego.frenet.valid;
    return ego;
  };
  const auto make_peer = [&frame](const std::string &id, double s_m, double d_m,
                                  double yaw_error_rad, double speed_mps) {
    const auto pose = frame.frenetToCartesian(s_m, d_m);
    sl::OpponentState peer;
    peer.id = id;
    peer.x = pose.x;
    peer.y = pose.y;
    peer.yaw = pose.yaw + yaw_error_rad;
    for (int i = 0; i < 10; ++i) {
      const auto location = frame.project(peer.x, peer.y, peer.yaw);
      peer.yaw = frame.interpolate(location.s).yaw + yaw_error_rad;
    }
    peer.speed_mps = speed_mps;
    peer.vx_mps = speed_mps * std::cos(peer.yaw);
    peer.vy_mps = speed_mps * std::sin(peer.yaw);
    peer.stamp_sec = now_sec;
    peer.uncertainty_x_m = 0.15;
    peer.uncertainty_y_m = 0.15;
    peer.frenet = frame.project(peer.x, peer.y, peer.yaw);
    peer.valid = peer.frenet.valid;
    return peer;
  };

  // Captured v2 first-ambiguity geometry: the track-heading error reaches
  // 0.6432 rad and the relative-heading error reaches about 0.50 rad, while
  // both vehicles still make finite positive tangent progress. The helper's
  // D1 yaw offset compensates for projecting the captured Frenet pose back
  // through the discretized CSV so the resulting diagnostic is 0.6432 rad.
  const auto d2_ego =
      make_ego(29.935501818, -1.335929480, -0.155909776, 0.174692811);
  const auto d1_peer =
      make_peer("d1", 31.074877600, 1.373569766, -0.621102873, 1.310196081);
  sl::LatticePlanner d2_planner(d2_config, &frame, &map);
  const auto d2_output = d2_planner.update(d2_ego, {d1_peer}, true, now_sec);
  EXPECT_FALSE(d2_output.preventive_side_role_active)
      << d2_output.reason << " ego_heading="
      << d2_output.preventive_side_role_eligibility_diagnostic
             .ego_track_heading_error_rad
      << " peer_heading="
      << d2_output.preventive_side_role_eligibility_diagnostic
             .peer_track_heading_error_rad
      << " relative="
      << d2_output.preventive_side_role_eligibility_diagnostic
             .relative_heading_error_rad;
  EXPECT_FALSE(d2_output.preventive_side_role_leader);

  const auto d1_ego =
      make_ego(31.074877600, 1.373569766, -0.621102873, 1.310196081);
  const auto d2_peer =
      make_peer("d2", 29.935501818, -1.335929480, -0.155909776, 0.174692811);
  sl::LatticePlanner d1_planner(d1_config, &frame, &map);
  const auto d1_output = d1_planner.update(d1_ego, {d2_peer}, true, now_sec);
  EXPECT_FALSE(d1_output.preventive_side_role_active)
      << d1_output.reason << " ego_heading="
      << d1_output.preventive_side_role_eligibility_diagnostic
             .ego_track_heading_error_rad
      << " peer_heading="
      << d1_output.preventive_side_role_eligibility_diagnostic
             .peer_track_heading_error_rad
      << " relative="
      << d1_output.preventive_side_role_eligibility_diagnostic
             .relative_heading_error_rad;
  EXPECT_FALSE(d1_output.preventive_side_role_leader);
}

TEST(LatticePlanner,
     PreventiveSideRoleRejectsStationaryNegativeProgressAndCounterflow) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  constexpr double now_sec = 2.5;
  const auto ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, now_sec);

  auto stationary = sideRoleOpponent(frame, "d2", 10.0, 1.95, 0.0, now_sec);
  sl::LatticePlanner stationary_planner(config, &frame, &map);
  const auto stationary_output =
      stationary_planner.update(ego, {stationary}, true, now_sec);
  EXPECT_FALSE(stationary_output.preventive_side_role_active);
  EXPECT_EQ(
      stationary_output.preventive_side_role_eligibility_diagnostic.failure,
      sl::PreventiveSideRoleEligibilityFailure::PEER_TANGENT_PROGRESS);

  auto negative_ego = ego;
  negative_ego.yaw = M_PI;
  negative_ego.frenet =
      frame.project(negative_ego.x, negative_ego.y, negative_ego.yaw);
  sl::LatticePlanner negative_planner(config, &frame, &map);
  const auto negative_output =
      negative_planner.update(negative_ego, {stationary}, true, now_sec);
  EXPECT_FALSE(negative_output.preventive_side_role_active);
  EXPECT_EQ(negative_output.preventive_side_role_eligibility_diagnostic.failure,
            sl::PreventiveSideRoleEligibilityFailure::EGO_TRACK_HEADING);

  auto counterflow = sideRoleOpponent(frame, "d2", 10.0, 1.95, 2.0, now_sec);
  counterflow.yaw = M_PI;
  counterflow.vx_mps = -2.0;
  counterflow.vy_mps = 0.0;
  counterflow.frenet =
      frame.project(counterflow.x, counterflow.y, counterflow.yaw);
  sl::LatticePlanner counterflow_planner(config, &frame, &map);
  const auto counterflow_output =
      counterflow_planner.update(ego, {counterflow}, true, now_sec);
  EXPECT_FALSE(counterflow_output.preventive_side_role_active);
  EXPECT_EQ(
      counterflow_output.preventive_side_role_eligibility_diagnostic.failure,
      sl::PreventiveSideRoleEligibilityFailure::PEER_TRACK_HEADING);
}

TEST(LatticePlanner,
     PreventiveSideRoleUsesCartesianHardClearanceNotFrenetMonotonicity) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  const auto ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, 1.0);
  const auto peer = sideRoleOpponent(frame, "d2", 10.0, 1.95, 0.0, 1.0);

  sl::TrajectoryPoint initial;
  initial.x = ego.x;
  initial.y = ego.y;
  initial.yaw = ego.yaw;
  initial.s = ego.frenet.s;
  initial.d = ego.frenet.d;
  initial.time_sec = 0.0;
  auto recovered = initial;
  recovered.x = 14.0;
  recovered.y = 0.248;
  recovered.s = 14.0;
  recovered.d = 0.248;
  recovered.time_sec = 1.0;
  auto decreasing = recovered;
  decreasing.x = 10.0;
  decreasing.s = 10.0;

  sl::LatticePlanner planner(config, &frame, &map);
  sl::PreventiveSideRoleCandidateDiagnostic accepted_diagnostic;
  EXPECT_TRUE(planner.validateSideRolePeerSeparation(
      ego, peer, {initial, recovered}, &accepted_diagnostic));
  EXPECT_EQ(accepted_diagnostic.failure,
            sl::PreventiveSideRoleCandidateFailure::NONE);

  sl::PreventiveSideRoleCandidateDiagnostic decreasing_diagnostic;
  EXPECT_TRUE(planner.validateSideRolePeerSeparation(
      ego, peer, {initial, decreasing}, &decreasing_diagnostic));
  EXPECT_EQ(decreasing_diagnostic.failure,
            sl::PreventiveSideRoleCandidateFailure::NONE);

  auto hard = decreasing;
  hard.y = 0.50;
  hard.d = 0.50;
  sl::PreventiveSideRoleCandidateDiagnostic rejected_diagnostic;
  EXPECT_FALSE(planner.validateSideRolePeerSeparation(
      ego, peer, {initial, hard}, &rejected_diagnostic));
  EXPECT_EQ(rejected_diagnostic.failure,
            sl::PreventiveSideRoleCandidateFailure::DENSE_PEER_SEPARATION);
  EXPECT_EQ(rejected_diagnostic.violating_sample_index, 1);
  EXPECT_LT(rejected_diagnostic.observed_clearance_m,
            config.opponent_hard_clearance_m);
}

TEST(LatticePlanner,
     PreventiveSideRoleRechecksCartesianHardClearanceOnOutputHorizon) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  const auto ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, 1.0);
  const auto peer = sideRoleOpponent(frame, "d2", 10.0, 1.95, 2.0, 1.0);
  std::vector<double> speed(static_cast<std::size_t>(config.horizon_points),
                            2.0);
  std::vector<double> away(static_cast<std::size_t>(config.horizon_points),
                           -0.05);
  away.front() = ego.frenet.d;
  std::vector<double> inward(static_cast<std::size_t>(config.horizon_points),
                             0.50);
  inward.front() = ego.frenet.d;

  sl::LatticePlanner planner(config, &frame, &map);
  EXPECT_TRUE(planner.validateSideRoleOutputHorizon(ego, peer, {peer}, away,
                                                    speed, {}));
  sl::OutputHorizonDiagnostic output_diagnostic;
  sl::PreventiveSideRoleCandidateDiagnostic role_diagnostic;
  EXPECT_FALSE(planner.validateSideRoleOutputHorizon(
      ego, peer, {peer}, inward, speed, {}, &output_diagnostic,
      &role_diagnostic));
  EXPECT_TRUE(output_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              output_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);
}

TEST(LatticePlanner, PreventiveSideRoleTieIsNeutralAndNeedsDistinctGeometry) {
  auto frame = straightFrame();
  auto d1_config = sideRoleConfig("d1");
  auto map = openGrid(d1_config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, d1_config));
  auto d2_config = sideRoleConfig("d2");
  constexpr double now_sec = 2.0;
  const auto d1_ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, now_sec);
  const auto d2_ego = sideRoleEgo(frame, 10.0, 1.95, 2.0, now_sec);
  auto d2 = sideRoleOpponent(frame, "d2", 10.0, 1.95, 2.0, now_sec);
  auto d1 = sideRoleOpponent(frame, "d1", 10.0, 0.0, 2.0, now_sec);
  d2.vy_mps = -0.20;
  d1.vy_mps = 0.20;

  sl::LatticePlanner d1_planner(d1_config, &frame, &map);
  sl::LatticePlanner d2_planner(d2_config, &frame, &map);
  const auto d1_output = d1_planner.update(d1_ego, {d2}, true, now_sec);
  const auto d2_output = d2_planner.update(d2_ego, {d1}, true, now_sec);
  ASSERT_TRUE(d1_output.preventive_side_role_active) << d1_output.reason;
  ASSERT_TRUE(d2_output.preventive_side_role_active) << d2_output.reason;
  EXPECT_TRUE(d1_output.preventive_side_role_neutral);
  EXPECT_TRUE(d2_output.preventive_side_role_neutral);
  EXPECT_EQ(d1_output.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::NEUTRAL_HOLD);
  EXPECT_EQ(d2_output.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::NEUTRAL_HOLD);

  const auto duplicate = d1_planner.update(d1_ego, {d2}, true, now_sec);
  EXPECT_EQ(duplicate.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::NEUTRAL_HOLD);
  sl::PlannerOutput resolved;
  for (int sample = 1;
       sample <= d1_config.preventive_side_role_role_confirm_samples;
       ++sample) {
    const double stamp_sec = now_sec + sample * 0.1;
    auto ahead = sideRoleOpponent(frame, "d2", 10.5, 1.95, 1.0, stamp_sec);
    ahead.vy_mps = -0.20;
    resolved = d1_planner.update(sideRoleEgo(frame, 10.0, 0.0, 1.0, stamp_sec),
                                 {ahead}, true, stamp_sec);
  }
  EXPECT_EQ(resolved.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::FOLLOWER_YIELD);
  EXPECT_FALSE(resolved.preventive_side_role_neutral);
}

TEST(LatticePlanner,
     PreventiveSideRoleUsesSynchronizedCartesianAndRejectsRoleReversal) {
  auto frame = straightFrame();
  auto d1_config = sideRoleConfig("d1");
  auto map = openGrid(d1_config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, d1_config));
  auto d2_config = sideRoleConfig("d2");

  auto d1_ego = sideRoleEgo(frame, 10.0, 0.0, 1.0, 10.0);
  auto d2_from_d1 = sideRoleOpponent(frame, "d2", 11.07, 2.05, 0.4, 10.0);
  d2_from_d1.vy_mps = -0.30;
  sl::LatticePlanner d1_planner(d1_config, &frame, &map);
  const auto d1_entry = d1_planner.update(d1_ego, {d2_from_d1}, true, 10.0);
  ASSERT_TRUE(d1_entry.preventive_side_role_active) << d1_entry.reason;
  EXPECT_NEAR(d1_entry.preventive_side_role_eligibility_diagnostic.delta_s_m,
              1.07, 1.0e-6);
  EXPECT_FALSE(d1_entry.preventive_side_role_leader);
  EXPECT_EQ(d1_entry.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::FOLLOWER_YIELD);

  auto d2_ego = sideRoleEgo(frame, 10.122, 2.05, 0.8, 10.0);
  auto d1_from_d2 = sideRoleOpponent(frame, "d1", 10.0, 0.0, 1.3, 10.0);
  d1_from_d2.vy_mps = 0.30;
  sl::LatticePlanner d2_planner(d2_config, &frame, &map);
  const auto d2_entry = d2_planner.update(d2_ego, {d1_from_d2}, true, 10.0);
  ASSERT_TRUE(d2_entry.preventive_side_role_active) << d2_entry.reason;
  EXPECT_NEAR(d2_entry.preventive_side_role_eligibility_diagnostic.delta_s_m,
              -0.122, 1.0e-6);
  EXPECT_TRUE(d2_entry.preventive_side_role_neutral);
  EXPECT_EQ(d2_entry.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::NEUTRAL_HOLD);

  d1_ego = sideRoleEgo(frame, 11.5, 0.0, 0.0, 10.1);
  d2_from_d1 = sideRoleOpponent(frame, "d2", 10.5, 2.05, 0.0, 10.1);
  const auto sign_flipped = d1_planner.update(d1_ego, {d2_from_d1}, true, 10.1);
  EXPECT_EQ(sign_flipped.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(sign_flipped.reason, "preventive_side_role_geometry_changed");
  EXPECT_EQ(sign_flipped.preventive_side_role_eligibility_diagnostic.failure,
            sl::PreventiveSideRoleEligibilityFailure::ROLE_CHANGED);
}

TEST(LatticePlanner,
     PreventiveSideRoleEntersPredictiveLateralAdjacencyBeforeSideOverlap) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  const auto ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, 20.0);
  auto peer = sideRoleOpponent(frame, "d2", 12.8, 2.05, 1.0, 20.0);
  peer.vy_mps = -0.40;

  sl::LatticePlanner planner(config, &frame, &map);
  const auto output = planner.update(ego, {peer}, true, 20.0);
  ASSERT_TRUE(output.preventive_side_role_active) << output.reason;
  EXPECT_TRUE(
      output.preventive_side_role_eligibility_diagnostic.ego_relation !=
          sl::CurrentPoseOpponentRelation::SIDE_OVERLAP ||
      output.preventive_side_role_eligibility_diagnostic.peer_relation !=
          sl::CurrentPoseOpponentRelation::SIDE_OVERLAP);
  EXPECT_LT(output.preventive_side_role_eligibility_diagnostic
                .predicted_time_to_hard_sec,
            config.preventive_side_role_prediction_horizon_sec);
  EXPECT_EQ(output.preventive_side_role_eligibility_diagnostic.entry_reason,
            "cv_predicted_hard_clearance");

  auto far = peer;
  far.y = 4.5;
  far.frenet = frame.project(far.x, far.y, far.yaw);
  sl::LatticePlanner far_planner(config, &frame, &map);
  EXPECT_FALSE(
      far_planner.update(ego, {far}, true, 20.0).preventive_side_role_active);

  auto nonapproaching = sideRoleOpponent(frame, "d2", 11.0, 2.05, 2.0, 20.0);
  sl::LatticePlanner nonapproaching_planner(config, &frame, &map);
  const auto no_role =
      nonapproaching_planner.update(ego, {nonapproaching}, true, 20.0);
  EXPECT_FALSE(no_role.preventive_side_role_active);
  EXPECT_EQ(no_role.preventive_side_role_eligibility_diagnostic.failure,
            sl::PreventiveSideRoleEligibilityFailure::PREDICTED_HARD_CLEARANCE);
}

TEST(LatticePlanner,
     PreventiveSideRoleLeaderPrefersStraightThenAwayAndNeverTowardPeer) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);

  const auto ego = sideRoleEgo(frame, 10.5, 0.0, 1.0, 30.0);
  auto peer = sideRoleOpponent(frame, "d2", 10.0, 2.05, 0.8, 30.0);
  peer.vy_mps = -0.30;
  const auto output = planner.update(ego, {peer}, true, 30.0);
  ASSERT_TRUE(output.preventive_side_role_active) << output.reason;
  EXPECT_TRUE(output.preventive_side_role_leader);
  EXPECT_EQ(output.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::LEADER_PROCEED);
  EXPECT_NE(output.preventive_side_role_action,
            sl::PreventiveSideRoleAction::LEFT_ESCAPE);
  if (output.safe_lateral) {
    EXPECT_TRUE(output.preventive_side_role_action ==
                    sl::PreventiveSideRoleAction::STRAIGHT_CURRENT_CORRIDOR ||
                output.preventive_side_role_action ==
                    sl::PreventiveSideRoleAction::RIGHT_ESCAPE);
    ASSERT_TRUE(planner.selected().has_value());
    double body_left_displacement_m = 0.0;
    for (const auto &point : planner.selected()->dense) {
      const auto current_corridor =
          frame.frenetToCartesian(point.s, ego.frenet.d);
      const double projected_m =
          -(point.x - current_corridor.x) * std::sin(ego.yaw) +
          (point.y - current_corridor.y) * std::cos(ego.yaw);
      if (std::abs(projected_m) > std::abs(body_left_displacement_m)) {
        body_left_displacement_m = projected_m;
      }
    }
    if (output.preventive_side_role_action ==
        sl::PreventiveSideRoleAction::RIGHT_ESCAPE) {
      EXPECT_LT(body_left_displacement_m, -0.05);
    } else {
      EXPECT_LE(std::abs(body_left_displacement_m), 0.05);
    }
  } else {
    EXPECT_EQ(output.preventive_side_role_action,
              sl::PreventiveSideRoleAction::SAFE_STOP);
  }
}

TEST(LatticePlanner,
     PreventiveSideRoleExitUsesFreshHysteresisAndNeverSwitchesPeer) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d2");
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  auto ego = sideRoleEgo(frame, 10.0, 2.05, 2.0, 40.0);
  auto peer = sideRoleOpponent(frame, "d1", 10.5, 0.0, 1.0, 40.0);
  peer.vy_mps = 0.30;

  sl::LatticePlanner release_planner(config, &frame, &map);
  const auto entry = release_planner.update(ego, {peer}, true, 40.0);
  ASSERT_EQ(entry.preventive_side_role_phase,
            sl::PreventiveSideRolePhase::FOLLOWER_YIELD);
  ego = sideRoleEgo(frame, 10.0, 2.05, 0.0, 40.1);
  const auto escaped_sample = [&](double stamp_sec) {
    auto escaped = sideRoleOpponent(frame, "d1", 15.5, 0.0, 1.0, stamp_sec);
    return release_planner.update(ego, {escaped}, true, stamp_sec);
  };
  EXPECT_TRUE(escaped_sample(40.1).preventive_side_role_active);
  EXPECT_TRUE(escaped_sample(40.2).preventive_side_role_active);
  EXPECT_FALSE(escaped_sample(40.3).preventive_side_role_active);

  sl::LatticePlanner switch_planner(config, &frame, &map);
  auto switch_peer = sideRoleOpponent(frame, "d1", 10.5, 0.0, 1.0, 41.0);
  switch_peer.vy_mps = 0.30;
  ASSERT_TRUE(switch_planner
                  .update(sideRoleEgo(frame, 10.0, 2.05, 2.0, 41.0),
                          {switch_peer}, true, 41.0)
                  .preventive_side_role_active);
  auto replacement = sideRoleOpponent(frame, "d3", 10.5, 0.0, 1.0, 41.1);
  replacement.vy_mps = 0.30;
  const auto switched = switch_planner.update(
      sideRoleEgo(frame, 10.0, 2.05, 0.0, 41.1), {replacement}, true, 41.1);
  EXPECT_EQ(switched.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(switched.reason, "preventive_side_role_peer_dropout");
  EXPECT_FALSE(switched.preventive_side_role_active);
}

TEST(LatticePlanner, PreventiveSideRoleHealthFaultClearsGeometryRoleAuthority) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  config.mpc_health_release_samples = 1;
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);
  const auto ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, 50.0);
  auto peer = sideRoleOpponent(frame, "d2", 10.5, 2.05, 1.0, 50.0);
  peer.vy_mps = -0.30;
  ASSERT_TRUE(
      planner.update(ego, {peer}, true, 50.0).preventive_side_role_active);

  sl::MpcHealthStatus fault;
  fault.valid = true;
  fault.infeasible_count = config.mpc_health_infeasible_count_threshold;
  fault.solve_time_ms = 0.0;
  fault.age_sec = 0.0;
  fault.sample_sequence = 1U;
  peer.stamp_sec = 50.1;
  const auto interrupted = planner.update(
      sideRoleEgo(frame, 10.0, 0.0, 2.0, 50.1), {peer}, true, 50.1, fault);
  EXPECT_EQ(interrupted.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(interrupted.reason, "mpc_health_infeasible_guard");
  EXPECT_FALSE(interrupted.safe_lateral);
  EXPECT_TRUE(interrupted.lateral_offsets_m.empty());
  EXPECT_FALSE(interrupted.pass_continuation_suspended);
}

TEST(LatticePlanner, PreventiveSideRoleStopsWhenNoAllowedCandidateExists) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  config.adaptive_pass_offset_enabled = false;
  config.lateral_targets_m = {100.0, 101.0, 102.0, 103.0, 104.0};
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);

  auto ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, 35.0);
  auto peer = sideRoleOpponent(frame, "d2", 10.5, 2.05, 1.0, 35.0);
  peer.vy_mps = -0.30;
  ASSERT_TRUE(
      planner.update(ego, {peer}, true, 35.0).preventive_side_role_active);
  const auto output = planner.update(ego, {peer}, true, 35.0);
  EXPECT_EQ(output.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(output.preventive_side_role_action,
            sl::PreventiveSideRoleAction::SAFE_STOP);
  EXPECT_FALSE(output.safe_lateral);
  EXPECT_TRUE(output.lateral_offsets_m.empty());
}

TEST(LatticePlanner,
     PreventiveSideRoleFailsClosedAtHardThresholdAndOnAmbiguity) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  constexpr double now_sec = 3.0;
  const auto ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, now_sec);

  const auto below_hard =
      sideRoleOpponent(frame, "d2", 10.0, 1.69, 2.0, now_sec);
  sl::LatticePlanner hard_planner(config, &frame, &map);
  const auto hard = hard_planner.update(ego, {below_hard}, true, now_sec);
  EXPECT_EQ(hard.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(hard.reason, "current_pose_hard_collision");
  EXPECT_FALSE(hard.safe_lateral);

  auto unknown_config = config;
  unknown_config.own_vehicle_id = "auto";
  auto left = sideRoleOpponent(frame, "d2", 10.0, 1.95, 2.0, now_sec);
  left.vy_mps = -0.20;
  sl::LatticePlanner unknown_planner(unknown_config, &frame, &map);
  const auto unknown = unknown_planner.update(ego, {left}, true, now_sec);
  EXPECT_FALSE(unknown.preventive_side_role_active);
  EXPECT_EQ(unknown.preventive_side_role_eligibility_diagnostic.failure,
            sl::PreventiveSideRoleEligibilityFailure::INVALID_VEHICLE_ID);

  auto right = sideRoleOpponent(frame, "d3", 10.0, -1.95, 2.0, now_sec);
  right.vy_mps = 0.20;
  sl::LatticePlanner multiple_planner(config, &frame, &map);
  const auto multiple =
      multiple_planner.update(ego, {left, right}, true, now_sec);
  EXPECT_EQ(multiple.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(multiple.reason, "preventive_side_role_multiple_peers");
  EXPECT_EQ(multiple.preventive_side_role_eligibility_diagnostic.failure,
            sl::PreventiveSideRoleEligibilityFailure::MULTIPLE_PEERS);
  EXPECT_FALSE(multiple.safe_lateral);

  sl::LatticePlanner duplicate_planner(config, &frame, &map);
  const auto duplicate =
      duplicate_planner.update(ego, {left, left}, true, now_sec);
  EXPECT_EQ(duplicate.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(duplicate.reason, "preventive_side_role_ambiguous_peer");
  EXPECT_EQ(duplicate.preventive_side_role_eligibility_diagnostic.failure,
            sl::PreventiveSideRoleEligibilityFailure::DUPLICATE_PEER_ID);
  EXPECT_FALSE(duplicate.safe_lateral);

  auto occupied_map = testGrid(config, true);
  ASSERT_TRUE(occupied_map.buildReferenceLayer(frame, config));
  sl::LatticePlanner wall_planner(config, &frame, &occupied_map);
  const auto wall = wall_planner.update(ego, {left}, true, now_sec);
  EXPECT_EQ(wall.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(wall.reason, "current_pose_hard_collision");
  EXPECT_EQ(wall.current_collision_kind, sl::CollisionKind::WALL);
  EXPECT_FALSE(wall.safe_lateral);
}

TEST(LatticePlanner,
     PreventiveSideRoleDropoutAndHealthFaultCannotPreserveLateral) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  constexpr double now_sec = 4.0;
  const auto ego = sideRoleEgo(frame, 10.0, 0.0, 3.0, now_sec);
  auto peer = sideRoleOpponent(frame, "d2", 10.5, 1.95, 3.0, now_sec);
  peer.vy_mps = -0.20;

  sl::LatticePlanner dropout_planner(config, &frame, &map);
  const auto active = dropout_planner.update(ego, {peer}, true, now_sec);
  ASSERT_TRUE(active.preventive_side_role_active) << active.reason;
  const auto dropout = dropout_planner.update(ego, {}, true, now_sec + 0.05);
  EXPECT_EQ(dropout.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(dropout.reason, "preventive_side_role_peer_dropout");
  EXPECT_FALSE(dropout.safe_lateral);
  EXPECT_TRUE(dropout.lateral_offsets_m.empty());

  sl::LatticePlanner stale_planner(config, &frame, &map);
  const auto stale = stale_planner.update(ego, {peer}, false, now_sec);
  EXPECT_EQ(stale.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(stale.reason, "invalid_or_stale_input");

  auto front = sideRoleOpponent(frame, "d3", 13.0, 0.0, 0.0, now_sec);
  sl::LatticePlanner third_party_planner(config, &frame, &map);
  const auto third_party =
      third_party_planner.update(ego, {peer, front}, true, now_sec);
  EXPECT_EQ(third_party.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_NE(third_party.reason, "preventive_side_role_multiple_peers");
  EXPECT_TRUE(third_party.preventive_side_role_active);
  EXPECT_EQ(third_party.preventive_side_role_peer_id, "d2");
  EXPECT_FALSE(third_party.safe_lateral);

  sl::MpcHealthStatus unhealthy;
  unhealthy.valid = true;
  unhealthy.infeasible_count = config.mpc_health_infeasible_count_threshold;
  unhealthy.solve_time_ms = 0.0;
  unhealthy.age_sec = 0.0;
  unhealthy.sample_sequence = 1U;
  sl::LatticePlanner unhealthy_planner(config, &frame, &map);
  const auto guarded =
      unhealthy_planner.update(ego, {peer}, true, now_sec, unhealthy);
  EXPECT_EQ(guarded.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(guarded.reason, "mpc_health_infeasible_guard");
  EXPECT_FALSE(guarded.safe_lateral);
  EXPECT_TRUE(guarded.lateral_offsets_m.empty());
}

TEST(LatticePlanner,
     PreventiveSideRoleKeepsNoneligibleThirdInEscapeCollisionChecks) {
  auto frame = straightFrame();
  auto config = sideRoleConfig("d1");
  config.adaptive_pass_offset_enabled = false;
  config.lateral_targets_m = {0.0, 0.1, -0.1, 0.2, -0.2};
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  const auto run_until_escape = [&](bool include_front) {
    sl::LatticePlanner planner(config, &frame, &map);
    auto ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, 60.0);
    auto peer = sideRoleOpponent(frame, "d2", 10.5, 2.05, 1.0, 60.0);
    peer.vy_mps = -0.30;
    std::vector<sl::OpponentState> opponents{peer};
    if (include_front) {
      opponents.push_back(sideRoleOpponent(frame, "d3", 13.0, 0.0, 0.0, 60.0));
    }
    const auto entry = planner.update(ego, opponents, true, 60.0);
    EXPECT_TRUE(entry.preventive_side_role_active) << entry.reason;
    EXPECT_EQ(entry.preventive_side_role_peer_id, "d2");

    ego = sideRoleEgo(frame, 10.0, 0.0, 0.0, 60.1);
    sl::PlannerOutput output;
    for (double stamp_sec : {60.1, 60.2, 60.3}) {
      opponents.clear();
      opponents.push_back(
          sideRoleOpponent(frame, "d2", 10.5, 2.05, 0.0, stamp_sec));
      if (include_front) {
        opponents.push_back(
            sideRoleOpponent(frame, "d3", 13.0, 0.0, 0.0, stamp_sec));
      }
      output = planner.update(ego, opponents, true, stamp_sec);
    }
    EXPECT_EQ(output.preventive_side_role_phase,
              sl::PreventiveSideRolePhase::FOLLOWER_YIELD);
    return output;
  };

  const auto clear = run_until_escape(false);
  EXPECT_TRUE(clear.safe_lateral) << clear.reason;
  const auto blocked = run_until_escape(true);
  EXPECT_EQ(blocked.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(blocked.reason,
            "preventive_side_role_leader_wait_no_safe_candidate");
  EXPECT_FALSE(blocked.safe_lateral);
  EXPECT_NE(blocked.preventive_side_role_candidate_diagnostic.failure,
            sl::PreventiveSideRoleCandidateFailure::NONE);

  const auto ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, 61.0);
  auto side_peer = sideRoleOpponent(frame, "d2", 10.5, 2.05, 1.0, 61.0);
  side_peer.vy_mps = -0.30;
  const auto hard_front = sideRoleOpponent(frame, "d3", 12.0, 0.0, 0.0, 61.0);
  sl::LatticePlanner hard_planner(config, &frame, &map);
  const auto hard =
      hard_planner.update(ego, {side_peer, hard_front}, true, 61.0);
  EXPECT_EQ(hard.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(hard.reason, "current_pose_hard_collision");
  EXPECT_EQ(hard.current_collision_kind, sl::CollisionKind::OPPONENT);
  EXPECT_FALSE(hard.preventive_side_role_active);
}

TEST(LatticePlanner, PreventiveSideRoleSuspendsExistingPassContinuationLatch) {
  auto frame = sampledSquareFrame();
  auto config = sideRoleConfig("d1");
  config.front_detection_radius_m = 20.0;
  config.experimental_exact_spatial_follow_shadow_enabled = false;
  config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  sl::applyResolvedControllerTrackabilityEnvelope(&config, 0.5236, 0.35);
  auto map = openGrid(config);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  auto ego = sideRoleEgo(frame, 2.0, 0.0, 5.0, 5.0);
  auto front = sideRoleOpponent(frame, "d2", 18.0, 0.0, 0.0, 5.0);
  sl::LatticePlanner planner(config, &frame, &map);
  const auto pass = planner.update(ego, {front}, true, 5.0);
  ASSERT_TRUE(pass.mode == sl::BehaviorMode::OVERTAKE_LEFT ||
              pass.mode == sl::BehaviorMode::OVERTAKE_RIGHT)
      << pass.reason;
  ASSERT_TRUE(pass.pass_continuation_latched);

  ego = sideRoleEgo(frame, 10.0, 0.0, 2.0, 5.1);
  auto peer = sideRoleOpponent(frame, "d3", 10.0, 1.95, 2.0, 5.1);
  peer.vy_mps = -0.20;
  const auto role = planner.update(ego, {peer}, true, 5.1);
  ASSERT_TRUE(role.preventive_side_role_active) << role.reason;
  EXPECT_EQ(role.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_FALSE(role.safe_lateral);
  EXPECT_TRUE(role.lateral_offsets_m.empty());
  EXPECT_TRUE(role.pass_continuation_latched);
  EXPECT_FALSE(role.pass_continuation_active);
  EXPECT_TRUE(role.pass_continuation_suspended);
  EXPECT_EQ(role.pass_continuation_target_id, "d2");
  EXPECT_EQ(role.target_id, "d3");
}

TEST(LatticePlanner, UnsafeDebugModeBypassesEnvironmentalSafetyEvaluation) {
  sl::PlannerConfig config;
  config.safety_evaluation_enabled = false;
  config.overtake_permission_profile_enabled = true;
  config.default_overtake_allowed = false;
  auto frame = sampledSquareFrame();
  auto map = testGrid(config, true);
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));
  sl::LatticePlanner planner(config, &frame, &map);

  sl::EgoState ego;
  ego.x = 2.0;
  ego.y = 0.0;
  ego.yaw = 0.0;
  ego.speed_mps = 2.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = true;

  sl::OpponentState opponent;
  opponent.id = "d2";
  // Keep the opponent close enough for footprint overlap while placing it
  // ahead so the front detector activates the lattice planner.
  opponent.x = ego.x + 1.0;
  opponent.y = ego.y;
  opponent.yaw = ego.yaw;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = true;

  sl::MpcHealthStatus unhealthy;
  unhealthy.valid = true;
  unhealthy.infeasible_count = 100;
  unhealthy.solve_time_ms = 1000.0;
  unhealthy.age_sec = 100.0;
  unhealthy.sample_sequence = 1U;

  const auto output = planner.update(ego, {opponent}, false, 10.0, unhealthy);
  EXPECT_NE(output.reason, "invalid_or_stale_input");
  EXPECT_NE(output.reason, "current_pose_hard_collision");
  EXPECT_NE(output.mode, sl::BehaviorMode::SPEED_GUARD);
  EXPECT_FALSE(output.mpc_health_guard_active);
  EXPECT_TRUE(output.overtake_permission_allowed);
  EXPECT_EQ(output.overtake_permission_reason, "safety_evaluation_disabled");
  EXPECT_GT(output.generated_candidates, 0);
  EXPECT_TRUE(
      std::none_of(planner.candidates().begin(), planner.candidates().end(),
                   [](const auto &candidate) {
                     return candidate.rejection_reason == "hard_collision";
                   }));
}

TEST(LatticePlanner, ExactCartesianRun09SelectsEvaluatedExecutionGeometry) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};
  config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  config.hard_max_steer_rad = 0.64;
  config.planner_max_steer_rad = 0.64;
  config.max_steer_rate_radps = 0.5;
  config.exact_cartesian_execution_enabled = true;
  // Isolate the execution-geometry contract from the independently calibrated
  // production cost-stop boundary (the default run09 cost is 460 vs 380).
  const std::string share = TEST_MPC_SOURCE_DIR;
  sl::FrenetFrame frame;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      share + "/env/final_ver3/traj_mincurv_manual.csv", &error)) << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(share + "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  sl::EgoState ego;
  ego.x = 89633.252;
  ego.y = 43124.957;
  ego.yaw = 2.264;
  ego.speed_mps = 0.0;
  ego.stamp_sec = 3.25;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.curvature = frame.interpolate(ego.frenet.s).kappa;
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);
  const auto opponent = [&](const char *id, double x, double y) {
    sl::OpponentState value;
    value.id = id;
    value.x = x;
    value.y = y;
    value.frenet = frame.project(x, y, 0.0);
    value.yaw = frame.interpolate(value.frenet.s).yaw;
    value.frenet = frame.project(x, y, value.yaw);
    value.stamp_sec = ego.stamp_sec;
    value.uncertainty_x_m = 0.15;
    value.uncertainty_y_m = 0.15;
    value.valid = value.frenet.valid;
    return value;
  };
  const std::vector<sl::OpponentState> opponents{
      opponent("d2", 89628.9140625, 43131.0),
      opponent("d3", 89624.7265625, 43137.74609375),
      opponent("d4", 89620.234375, 43144.671875)};

  sl::LatticePlanner planner(config, &frame, &map);
  const auto output = planner.update(ego, opponents, true, ego.stamp_sec);
  int feasible_total = std::numeric_limits<int>::max();
  int feasible_reference = -1;
  int feasible_wall = -1;
  int feasible_object = -1;
  for (const auto &candidate : planner.candidates()) {
    if (candidate.feasible && candidate.total_cost < feasible_total) {
      feasible_total = candidate.total_cost;
      feasible_reference = candidate.reference_cost;
      feasible_wall = candidate.wall_cost;
      feasible_object = candidate.object_cost;
    }
  }
  ASSERT_TRUE(planner.selected().has_value())
      << output.reason << " failure="
      << static_cast<int>(output.output_horizon_diagnostic.failure)
      << " waypoint=" << output.output_horizon_diagnostic.waypoint_index
      << " cost=" << feasible_total << " ref=" << feasible_reference
      << " wall=" << feasible_wall << " object=" << feasible_object
      << " stop=" << config.stop_cost;
  EXPECT_TRUE(output.active);
  EXPECT_TRUE(output.safe_lateral);
  EXPECT_FALSE(output.emergency_stop);
  EXPECT_EQ(output.execution_geometry_kind,
            sl::ExecutionGeometryKind::EXACT_CARTESIAN);
  ASSERT_EQ(output.lateral_offsets_m.size(), 2U);
  EXPECT_EQ(output.lateral_offsets_m, std::vector<double>({0.0, 0.0}));
  ASSERT_EQ(output.longitudinal_offsets_m.size(), 2U);
  EXPECT_DOUBLE_EQ(output.longitudinal_offsets_m.front(), 0.0);
  EXPECT_GT(output.longitudinal_offsets_m.back(), 0.0);
  EXPECT_TRUE(planner.selected()->feasible);
  EXPECT_LT(planner.selected()->safety_cost, config.stop_cost);
  EXPECT_GE(planner.selected()->total_cost, config.stop_cost);
}

TEST(LatticePlanner, RealMapExactOvertakeFixtureSelectsCartesianExecution) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};
  config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  sl::applyResolvedControllerTrackabilityEnvelope(&config, 0.5236, 0.35);
  config.exact_cartesian_execution_enabled = true;
  config.overtake_permission_profile_enabled = false;
  config.default_overtake_allowed = true;
  sl::FrenetFrame frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(
      std::string(TEST_MPC_SOURCE_DIR) +
          "/env/final_ver3/traj_mincurv_manual.csv",
      &error)) << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(
      std::string(TEST_MPC_SOURCE_DIR) +
          "/env/final_ver3/occupancy_grid_map.yaml",
      config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(frame, config));

  constexpr std::size_t kReferenceIndex = 25U;
  constexpr double kOpponentGapM = 7.5;
  ASSERT_LT(kReferenceIndex, frame.points().size());
  const double ego_s = frame.points()[kReferenceIndex].s;
  const auto ego_pose = frame.frenetToCartesian(ego_s, 0.0);
  const auto opponent_pose =
      frame.frenetToCartesian(ego_s + kOpponentGapM, 0.0);
  sl::EgoState ego;
  ego.x = ego_pose.x;
  ego.y = ego_pose.y;
  ego.yaw = ego_pose.yaw;
  ego.speed_mps = 1.0;
  ego.stamp_sec = 10.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.curvature = frame.interpolate(ego.frenet.s).kappa;
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);
  sl::OpponentState opponent;
  opponent.id = "d2";
  opponent.x = opponent_pose.x;
  opponent.y = opponent_pose.y;
  opponent.yaw = opponent_pose.yaw;
  opponent.stamp_sec = ego.stamp_sec;
  opponent.uncertainty_x_m = 0.30;
  opponent.uncertainty_y_m = 0.30;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = opponent.frenet.valid;
  ASSERT_TRUE(opponent.valid);

  sl::LatticePlanner planner(config, &frame, &map, &output_frame);
  const auto output = planner.update(ego, {opponent}, true, ego.stamp_sec);
  EXPECT_TRUE(output.active) << output.reason;
  EXPECT_EQ(output.mode, sl::BehaviorMode::OVERTAKE_RIGHT) << output.reason;
  EXPECT_TRUE(output.safe_lateral);
  EXPECT_EQ(output.execution_geometry_kind,
            sl::ExecutionGeometryKind::EXACT_CARTESIAN);
  ASSERT_TRUE(planner.selected().has_value());
  EXPECT_TRUE(planner.selected()->feasible);
  sl::OutputHorizonDiagnostic wire_diagnostic;
  const auto wire_execution = planner.boundedExactCartesianExecution(
      planner.selected().value(), {opponent}, 256U, &wire_diagnostic);
  ASSERT_TRUE(wire_execution.has_value())
      << sl::toString(wire_diagnostic.failure);
  EXPECT_LE(wire_execution->dense.size(), 256U);
  EXPECT_DOUBLE_EQ(wire_execution->dense.front().x,
                   planner.selected()->dense.front().x);
  EXPECT_DOUBLE_EQ(wire_execution->dense.back().x,
                   planner.selected()->dense.back().x);
}

TEST(LatticePlanner,
     Runtime10Generation489UsesContinuousTwoFrameProjection) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.2, -2.2};
  config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  sl::applyResolvedControllerTrackabilityEnvelope(&config, 0.5236, 0.35);
  config.exact_cartesian_execution_enabled = true;
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  config.overtake_permission_profile_enabled = false;
  config.default_overtake_allowed = true;
  config.front_enter_cycles = 1;
  config.normal_mode_min_hold_sec = 0.0;

  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(
      std::string(TEST_MPC_SOURCE_DIR) +
          "/env/final_ver3/traj_mincurv_manual.csv",
      &error)) << error;
  sl::GridMap map;
  ASSERT_TRUE(map.load(
      std::string(TEST_MPC_SOURCE_DIR) +
          "/env/final_ver3/occupancy_grid_map.yaml",
      config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  sl::EgoState ego;
  ego.x = 89631.4184286478;
  ego.y = 43127.920105601734;
  ego.yaw = 2.0829759361539733;
  ego.speed_mps = 0.22280558130669867;
  ego.yaw_rate_radps = 0.0005721452037306844;
  ego.stamp_sec = 30.924999308;
  ego.frenet = planning_frame.project(ego.x, ego.y, ego.yaw);
  ego.curvature = planning_frame.interpolate(ego.frenet.s).kappa;
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  const auto make_opponent = [&planning_frame, &ego](
                                 const std::string &id, double x, double y) {
    sl::OpponentState opponent;
    opponent.id = id;
    opponent.x = x;
    opponent.y = y;
    const auto projected = planning_frame.project(x, y, ego.yaw);
    opponent.yaw = projected.valid
                       ? planning_frame.interpolate(projected.s).yaw
                       : ego.yaw;
    opponent.speed_mps = id == "d2" ? 0.20 : 0.0;
    opponent.vx_mps = opponent.speed_mps * std::cos(opponent.yaw);
    opponent.vy_mps = opponent.speed_mps * std::sin(opponent.yaw);
    opponent.stamp_sec = ego.stamp_sec;
    opponent.uncertainty_x_m = 0.005;
    opponent.uncertainty_y_m = 0.005;
    opponent.frenet = projected;
    opponent.valid = opponent.frenet.valid;
    return opponent;
  };
  const std::vector<sl::OpponentState> opponents = {
      make_opponent("d2", 89628.9140625, 43131.0),
      make_opponent("d3", 89624.7265625, 43137.74609375),
      make_opponent("d4", 89620.234375, 43144.671875),
  };
  ASSERT_TRUE(std::all_of(opponents.begin(), opponents.end(),
                          [](const auto &opponent) {
                            return opponent.valid;
                          }));

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto candidates = planner.generateCandidates(ego, opponents);
  ASSERT_FALSE(candidates.empty());
  std::optional<sl::CandidateTrajectory> matching_candidate;
  for (const auto &candidate : candidates) {
    if (candidate.feasible && std::abs(candidate.goal_d_m + 1.0) <= 1.0e-9 &&
        std::abs(candidate.tangent_scale - 1.0) <= 1.0e-9) {
      matching_candidate = candidate;
      break;
    }
  }
  ASSERT_TRUE(matching_candidate.has_value());
  std::vector<double> projected_d;
  std::vector<double> projected_speed;
  std::vector<double> projected_longitudinal_offsets_m;
  sl::OutputHorizonDiagnostic projected_diagnostic;
  EXPECT_TRUE(sl::StateLatticeTestAccess::outputHorizon(
      planner, ego, matching_candidate.value(), opponents,
      config.safe_stop_speed_mps, &projected_d, &projected_speed,
      &projected_longitudinal_offsets_m, &projected_diagnostic))
      << sl::toString(projected_diagnostic.failure) << " waypoint="
      << projected_diagnostic.waypoint_index << " observed="
      << projected_diagnostic.observed_value;
  EXPECT_EQ(projected_diagnostic.failure, sl::OutputHorizonFailure::NONE);
  ASSERT_EQ(projected_d.size(),
            static_cast<std::size_t>(config.horizon_points));
  ASSERT_EQ(projected_d.size(), projected_longitudinal_offsets_m.size());
  EXPECT_NEAR(projected_longitudinal_offsets_m.front(), 0.0, 1.0e-9);
  EXPECT_TRUE(std::is_sorted(projected_longitudinal_offsets_m.begin(),
                             projected_longitudinal_offsets_m.end()));

  sl::LatticePlanner same_frame_planner(config, &planning_frame, &map,
                                        &planning_frame);
  std::vector<double> same_frame_d;
  std::vector<double> same_frame_speed;
  std::vector<double> same_frame_longitudinal_offsets_m;
  sl::OutputHorizonDiagnostic same_frame_diagnostic;
  EXPECT_TRUE(sl::StateLatticeTestAccess::outputHorizon(
      same_frame_planner, ego, matching_candidate.value(), opponents,
      config.safe_stop_speed_mps, &same_frame_d, &same_frame_speed,
      &same_frame_longitudinal_offsets_m, &same_frame_diagnostic));
  EXPECT_EQ(same_frame_diagnostic.failure, sl::OutputHorizonFailure::NONE);
}
