#include "state_lattice_overtake_planner/frenet_frame.hpp"
#include "state_lattice_overtake_planner/lattice_planner.hpp"
#include "state_lattice_overtake_planner/reference_override_contract.hpp"

#include "fixtures/dev3_20260727_225451.hpp"
#include "fixtures/dev3_20260728_010706.hpp"
#include "fixtures/dev3_20260728_120212.hpp"
#include "fixtures/dev3_20260728_135436.hpp"
#include "fixtures/dev3_20260728_184222.hpp"
#include "fixtures/dev3_20260728_213709.hpp"
#include "fixtures/v26_gen470_plan.hpp"
#include "fixtures/v29_gen424_plan.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

namespace sl = state_lattice_overtake_planner;

namespace state_lattice_overtake_planner {
struct StateLatticeTestAccess {
  struct AcceptedPathConnectorSweepResult {
    std::size_t eligible_join_points{0U};
    std::size_t attempts{0U};
    std::size_t generation_rejects{0U};
    std::size_t evaluator_rejects{0U};
    std::size_t cartesian_rejects{0U};
    std::optional<CandidateTrajectory> accepted;
    std::size_t accepted_join_index{0U};
    double accepted_forward_arc_m{0.0};
    double accepted_tangent_scale{0.0};
  };

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

  static void seedPassContinuationLatch(
      LatticePlanner &planner, const std::string &target_id, int side,
      std::size_t lateral_index, std::size_t tangent_index, double goal_d_m,
      double required_arc_m, double target_observation_stamp_sec,
      std::uint64_t revision, bool profile_hint_valid = false,
      double profile_join_fraction = 0.0,
      double profile_yaw_magnitude = 0.0,
      double profile_tangent_scale = 1.0) {
    LatticePlanner::PassContinuationLatch latch;
    latch.target_id = target_id;
    latch.side = side;
    latch.lateral_index = lateral_index;
    latch.tangent_index = tangent_index;
    latch.goal_d_m = goal_d_m;
    latch.required_arc_m = required_arc_m;
    latch.target_observation_stamp_sec = target_observation_stamp_sec;
    latch.revision = revision;
    latch.clearance_profile_hint_valid = profile_hint_valid;
    latch.clearance_profile_join_fraction = profile_join_fraction;
    latch.clearance_profile_yaw_magnitude = profile_yaw_magnitude;
    latch.clearance_profile_tangent_scale = profile_tangent_scale;
    planner.pass_continuation_latch_ = latch;
    planner.active_ = true;
    planner.target_id_ = target_id;
    planner.previous_lateral_index_ = static_cast<int>(lateral_index);
    planner.previous_tangent_index_ = static_cast<int>(tangent_index);
    planner.last_mode_ = BehaviorMode::OVERTAKE_RIGHT;
  }

  static std::optional<std::pair<double, double>> clearanceProfileHint(
      const LatticePlanner &planner) {
    if (!planner.pass_continuation_latch_.has_value() ||
        !planner.pass_continuation_latch_->clearance_profile_hint_valid) {
      return std::nullopt;
    }
    return std::pair<double, double>{
        planner.pass_continuation_latch_->clearance_profile_join_fraction,
        planner.pass_continuation_latch_->clearance_profile_yaw_magnitude};
  }

  static bool hasPassContinuationAuthority(const LatticePlanner &planner) {
    return planner.pass_continuation_latch_.has_value() ||
           planner.suspended_pass_continuation_latch_.has_value();
  }

  static bool hasDeadlineProfileSearchHint(const LatticePlanner &planner) {
    return planner.deadline_profile_search_hint_.has_value();
  }

  static bool hasAcceptedPathContinuationHint(const LatticePlanner &planner) {
    return planner.accepted_path_continuation_hint_.has_value();
  }

  static void setStopCost(LatticePlanner &planner, double stop_cost) {
    planner.config_.stop_cost = stop_cost;
  }

  static void makeAcceptedPathContinuationJoinAmbiguous(
      LatticePlanner &planner) {
    if (!planner.accepted_path_continuation_hint_.has_value()) {
      return;
    }
    auto &dense =
        planner.accepted_path_continuation_hint_->candidate.dense;
    if (dense.size() > 30U) {
      dense[30U] = dense.front();
    }
  }

  static void seedAcceptedPathContinuationHint(
      LatticePlanner &planner, const std::vector<TrajectoryPoint> &dense,
      const std::string &target_id, int side, std::size_t lateral_index,
      std::size_t tangent_index, double goal_d_m,
      double target_observation_stamp_sec,
      double accepted_tangent_scale = 1.0) {
    CandidateTrajectory candidate;
    candidate.feasible = true;
    candidate.lateral_index = lateral_index;
    candidate.tangent_index = tangent_index;
    candidate.goal_d_m = goal_d_m;
    candidate.tangent_scale = accepted_tangent_scale;
    candidate.dense = dense;
    for (std::size_t i = 0U; i < candidate.dense.size(); ++i) {
      candidate.dense[i].u = static_cast<double>(i) /
                             static_cast<double>(candidate.dense.size() - 1U);
      if (i > 0U) {
        candidate.required_arc_m += std::hypot(
            candidate.dense[i].x - candidate.dense[i - 1U].x,
            candidate.dense[i].y - candidate.dense[i - 1U].y);
      }
    }
    for (std::size_t i = 0U; i < 5U; ++i) {
      const std::size_t index =
          i * (candidate.dense.size() - 1U) / 4U;
      candidate.representative.push_back(candidate.dense[index]);
    }
    planner.accepted_path_continuation_hint_ =
        LatticePlanner::AcceptedPathContinuationHint{
            target_id, side, lateral_index, tangent_index, goal_d_m,
            target_observation_stamp_sec, candidate, false};
    LatticePlanner::PassContinuationLatch latch;
    latch.target_id = target_id;
    latch.side = side;
    latch.lateral_index = lateral_index;
    latch.tangent_index = tangent_index;
    latch.goal_d_m = goal_d_m;
    latch.required_arc_m = candidate.required_arc_m;
    latch.target_observation_stamp_sec = target_observation_stamp_sec;
    latch.revision = 470U;
    planner.pass_continuation_latch_ = latch;
    planner.active_ = true;
    planner.target_id_ = target_id;
    planner.previous_lateral_index_ = static_cast<int>(lateral_index);
    planner.previous_tangent_index_ = static_cast<int>(tangent_index);
    planner.last_mode_ = BehaviorMode::OVERTAKE_RIGHT;
  }

  static AcceptedPathConnectorSweepResult sweepAcceptedPathConnectors(
      const LatticePlanner &planner, const EgoState &ego,
      const std::vector<OpponentState> &opponents, double minimum_tangent,
      double maximum_tangent, double tangent_step,
      double maximum_connector_arc_m = 1.00) {
    AcceptedPathConnectorSweepResult result;
    if (!planner.accepted_path_continuation_hint_.has_value() ||
        !std::isfinite(minimum_tangent) ||
        !std::isfinite(maximum_tangent) || !std::isfinite(tangent_step) ||
        !std::isfinite(maximum_connector_arc_m) || tangent_step <= 0.0 ||
        maximum_tangent < minimum_tangent || maximum_connector_arc_m < 0.30) {
      return result;
    }
    const auto &previous =
        planner.accepted_path_continuation_hint_->candidate;
    if (previous.dense.size() < 3U) {
      return result;
    }

    std::size_t nearest_index = 0U;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0U; i < previous.dense.size(); ++i) {
      const double distance =
          std::hypot(previous.dense[i].x - ego.x,
                     previous.dense[i].y - ego.y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = i;
      }
    }

    const double maximum_curvature =
        std::abs(std::tan(std::min(planner.config_.planner_max_steer_rad,
                                  planner.config_.hard_max_steer_rad)) /
                 planner.config_.wheel_base_m);
    const double curvature_limit =
        std::min(maximum_curvature,
                 planner.config_.reference_curvature_sanity_limit_radpm);
    const auto sanitize_curvature = [curvature_limit](double curvature) {
      return std::isfinite(curvature)
                 ? std::clamp(curvature, -curvature_limit, curvature_limit)
                 : 0.0;
    };

    constexpr double kMinimumConnectorArcM = 0.30;
    double forward_arc_m = 0.0;
    for (std::size_t join_index = nearest_index + 1U;
         join_index + 1U < previous.dense.size(); ++join_index) {
      forward_arc_m += std::hypot(
          previous.dense[join_index].x - previous.dense[join_index - 1U].x,
          previous.dense[join_index].y - previous.dense[join_index - 1U].y);
      if (!std::isfinite(forward_arc_m) ||
          forward_arc_m > maximum_connector_arc_m) {
        break;
      }
      if (forward_arc_m + planner.config_.tie_break_epsilon <
          kMinimumConnectorArcM) {
        continue;
      }
      ++result.eligible_join_points;
      const auto &join = previous.dense[join_index];
      const Pose2d join_pose{join.x, join.y, join.yaw};
      const std::size_t tangent_count = static_cast<std::size_t>(
          std::floor((maximum_tangent - minimum_tangent) / tangent_step +
                     0.5)) +
                                        1U;
      for (std::size_t tangent_index = 0U; tangent_index < tangent_count;
           ++tangent_index) {
        const double tangent_scale =
            minimum_tangent + static_cast<double>(tangent_index) * tangent_step;
        ++result.attempts;
        ParametricQuintic connector;
        if (!connector.configure(
                ego, sanitize_curvature(ego.curvature), join_pose,
                sanitize_curvature(join.kappa), tangent_scale)) {
          ++result.generation_rejects;
          continue;
        }
        CandidateTrajectory candidate = previous;
        candidate.tangent_scale = tangent_scale;
        candidate.dense = planner.denseSamples(connector, 40);
        if (candidate.dense.empty()) {
          ++result.generation_rejects;
          continue;
        }
        candidate.dense.insert(candidate.dense.end(),
                               previous.dense.cbegin() + join_index + 1U,
                               previous.dense.cend());
        if (candidate.dense.size() < 3U) {
          ++result.generation_rejects;
          continue;
        }
        candidate.required_arc_m = 0.0;
        for (std::size_t i = 0U; i < candidate.dense.size(); ++i) {
          candidate.dense[i].u = static_cast<double>(i) /
                                 static_cast<double>(candidate.dense.size() -
                                                     1U);
          if (i > 0U) {
            candidate.required_arc_m += std::hypot(
                candidate.dense[i].x - candidate.dense[i - 1U].x,
                candidate.dense[i].y - candidate.dense[i - 1U].y);
          }
        }
        candidate.representative.clear();
        for (std::size_t i = 0U; i < 5U; ++i) {
          const std::size_t index =
              i * (candidate.dense.size() - 1U) / 4U;
          candidate.representative.push_back(candidate.dense[index]);
        }
        if (!planner.evaluateTrajectory(&candidate, opponents,
                                        std::abs(ego.speed_mps))) {
          ++result.evaluator_rejects;
          continue;
        }
        OutputHorizonDiagnostic output_diagnostic;
        const auto bounded = planner.boundedExactCartesianExecution(
            candidate, opponents, 256U, &output_diagnostic);
        if (!bounded.has_value()) {
          ++result.cartesian_rejects;
          continue;
        }
        result.accepted = *bounded;
        result.accepted_join_index = join_index;
        result.accepted_forward_arc_m = forward_arc_m;
        result.accepted_tangent_scale = tangent_scale;
        return result;
      }
    }
    return result;
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

TEST(LowSpeedCurvatureContinuity,
     UsesBoundedPoseDeltaWithoutDividingByLowSpeed) {
  sl::LowSpeedCurvatureContinuity continuity;
  const sl::Pose2d generation701{89631.22177747132, 43128.78490166712,
                                1.91382787664883};
  const sl::Pose2d generation702{89631.2194331941, 43128.79145403971,
                                1.9128920022536398};

  EXPECT_FALSE(continuity
                   .update(generation701, 43.334999031, 0.04298023937091172,
                           -0.004549024714772127, 0.6, 0.5)
                   .has_value());
  const auto curvature = continuity.update(
      generation702, 43.474999028, 0.05384848386180184,
      -0.00734367215593885, 0.6, 0.5);
  ASSERT_TRUE(curvature.has_value());
  EXPECT_NEAR(curvature.value(), -0.1344818943, 1.0e-8);
}

TEST(LowSpeedCurvatureContinuity, RejectsUnusableOrDiscontinuousEvidence) {
  sl::LowSpeedCurvatureContinuity continuity;
  const sl::Pose2d anchor{10.0, 20.0, 0.1};
  EXPECT_FALSE(continuity.update(anchor, 1.0, 0.05, 0.0, 0.6, 0.5)
                   .has_value());

  // Travel below the deterministic baseline cannot amplify tiny yaw noise.
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{10.0001, 20.0, 0.1001}, 1.1, 0.05,
                           0.001, 0.6, 0.5)
                   .has_value());
  // Stale, non-finite, backwards-clock, and excessive-curvature evidence is
  // rejected rather than cached for a later planning cycle.
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{10.01, 20.0, 0.101}, 2.0, 0.05, 0.0,
                           0.6, 0.5)
                   .has_value());
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{NAN, 20.0, 0.1}, 2.1, 0.05, 0.0, 0.6,
                           0.5)
                   .has_value());
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{10.0, 20.0, 0.1}, 2.0, 0.05, 0.0, 0.6,
                           0.5)
                   .has_value());
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{10.01, 20.0, 0.3}, 2.1, 0.05, 0.0, 0.6,
                           0.5)
                   .has_value());
}

TEST(LowSpeedCurvatureContinuity,
     RejectedGeometricAndDirectSamplesCannotSeedNextEstimate) {
  sl::LowSpeedCurvatureContinuity continuity;
  const sl::Pose2d anchor{0.0, 0.0, 0.0};
  EXPECT_FALSE(continuity.update(anchor, 1.0, 0.05, 0.0, 0.6, 0.5)
                   .has_value());
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{0.01, 0.0, 0.2}, 1.1, 0.05, 0.0, 0.6,
                           0.5)
                   .has_value());
  // Rejection resets the estimator. This sample is only a new anchor and
  // cannot form a curvature estimate against the rejected pose.
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{0.02, 0.0, 0.201}, 1.2, 0.05, 0.0, 0.6,
                           0.5)
                   .has_value());

  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{0.03, 0.0, 0.201}, 1.3, 0.2, 1.0, 0.6,
                           0.5)
                   .has_value());
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{0.04, 0.0, 0.202}, 1.4, 0.05, 0.0, 0.6,
                           0.5)
                   .has_value());
}

TEST(LowSpeedCurvatureContinuity, BackwardsClockResetsLiveAnchor) {
  sl::LowSpeedCurvatureContinuity continuity;
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{0.0, 0.0, 0.0}, 2.0, 0.05, 0.0, 0.6,
                           0.5)
                   .has_value());
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{0.01, 0.0, 0.001}, 1.9, 0.05, 0.0, 0.6,
                           0.5)
                   .has_value());
  EXPECT_FALSE(continuity
                   .update(sl::Pose2d{0.02, 0.0, 0.002}, 2.1, 0.05, 0.0, 0.6,
                           0.5)
                   .has_value());
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
  // Keep this selector-reset fixture clear of the close-obstacle horizon
  // regression.  The test owns cost-triggered SAFE_STOP, not path shortening.
  target.x = 16.0;
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
  // With obstacle-distance clipping removed, every candidate retains its
  // nominal receding horizon and reaches the opponent check instead of being
  // compressed into the historical 1 m curvature/trackability failures.
  EXPECT_EQ(output.rejected_wall_candidates, 0);
  EXPECT_EQ(output.rejected_opponent_candidates, output.generated_candidates);
  EXPECT_EQ(output.rejected_curvature_candidates, 0);
  EXPECT_EQ(output.rejected_trackability_candidates, 0);
  EXPECT_EQ(output.rejected_other_candidates, 0);
  for (const auto &candidate : planner.candidates()) {
    EXPECT_GT(candidate.required_arc_m,
              config.minimum_obstacle_transition_distance_m);
  }

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
  EXPECT_EQ(pp_aligned_output.rejected_trackability_candidates, 0);
  EXPECT_EQ(pp_aligned_output.rejected_opponent_candidates,
            pp_aligned_output.generated_candidates);
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
     Dev3D1RecedingHorizonKeepsFullArcNearObstacle) {
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
      EXPECT_NEAR(candidate.required_arc_m, required_arc_m, 1.0e-6);
      EXPECT_NE(candidate.rejection_reason, "maximum_curvature");
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

TEST(LatticePlanner,
     CloseObstacleNominalHorizonPreservesTrackabilityProfileDecision) {
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
  const auto prepare = fast_planner.update(fast, {opponent}, true, 1.0);
  EXPECT_EQ(prepare.mode, sl::BehaviorMode::FOLLOW_BLOCKED) << prepare.reason;
  EXPECT_FALSE(prepare.safe_lateral);
  EXPECT_EQ(prepare.feasible_candidates, 0);
  EXPECT_GT(prepare.rejected_opponent_candidates, 0);
  EXPECT_FALSE(fast_planner.selected().has_value());

  sl::LatticePlanner pp_aligned_planner(pp_aligned_config, &frame, &map);
  const auto pp_aligned_pass =
      pp_aligned_planner.update(fast, {opponent}, true, 1.0);
  EXPECT_TRUE(pp_aligned_pass.mode == sl::BehaviorMode::OVERTAKE_LEFT ||
              pp_aligned_pass.mode == sl::BehaviorMode::OVERTAKE_RIGHT)
      << pp_aligned_pass.reason;
  EXPECT_TRUE(pp_aligned_pass.safe_lateral);
  EXPECT_GT(pp_aligned_pass.feasible_candidates, 0);
  EXPECT_GT(pp_aligned_pass.rejected_opponent_candidates, 0);
  ASSERT_TRUE(pp_aligned_planner.selected().has_value());
  EXPECT_GT(pp_aligned_planner.selected()->required_arc_m,
            pp_aligned_config.minimum_obstacle_transition_distance_m);

  sl::EgoState slow = fast;
  slow.speed_mps = config.safe_stop_speed_mps;
  sl::LatticePlanner slow_planner(config, &frame, &map);
  const auto pass = slow_planner.update(slow, {opponent}, true, 1.0);
  EXPECT_EQ(pass.mode, sl::BehaviorMode::FOLLOW_BLOCKED) << pass.reason;
  EXPECT_FALSE(pass.safe_lateral);
  EXPECT_FALSE(slow_planner.selected().has_value());
}

TEST(LatticePlanner,
     FullHorizonRejectsUnclearPassAndReportsNoSelectedSide) {
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
  EXPECT_EQ(output.mode, sl::BehaviorMode::FOLLOW_BLOCKED) << output.reason;
  EXPECT_FALSE(output.safe_lateral);
  EXPECT_GT(output.rejected_opponent_candidates, 0);
  EXPECT_FALSE(planner.selected().has_value());
  const auto &diagnostic =
      planner.lastPlanningCycleMetrics().pass_clearance_diagnostic;
  EXPECT_TRUE(diagnostic.evaluated);
  EXPECT_TRUE(diagnostic.inputs_valid);
  EXPECT_FALSE(diagnostic.observed_predicate);
  EXPECT_EQ(std::string(diagnostic.target_id.data()), target.id);
  EXPECT_EQ(diagnostic.side, 0);
  EXPECT_TRUE(std::isnan(diagnostic.actual_separation_m));
  EXPECT_TRUE(std::isnan(diagnostic.margin_m));
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
  ASSERT_EQ(output.mode, sl::BehaviorMode::SAFE_STOP) << output.reason;
  EXPECT_EQ(output.reason, "cost_stop_threshold");
  EXPECT_TRUE(planner.selected().has_value());
  EXPECT_DOUBLE_EQ(output.speed_cap_mps, config.safe_stop_speed_mps);
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
      make_ego(4.0), {make_opponent("front", 15.0, 0.0)}, true, 1.0);
  EXPECT_GT(pass_control.feasible_candidates, 0);
  EXPECT_TRUE(pass_control.mode == sl::BehaviorMode::OVERTAKE_LEFT ||
              pass_control.mode == sl::BehaviorMode::OVERTAKE_RIGHT)
      << pass_control.reason;

  // The matching cycle has a non-closing rear-only hard-clearance opponent.
  // Its future samples remain hard-checked, while its current pose cannot
  // authorize the otherwise available lateral maneuver.
  const auto output = rear_guarded_planner.update(
      make_ego(4.0),
      {make_opponent("front", 15.0, 0.0), make_opponent("rear", 1.586, 0.0)},
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
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      d1_planner, "d2", -1, 2U, 1U, -1.0, 8.0, now_sec, 1U,
      true, 0.34, 0.82, 1.0);
  d1_planner.clearPassContinuationAuthorityPreservingSearchHint();
  ASSERT_TRUE(sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(
      d1_planner));
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
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(
      d1_planner));

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
  // Reference deviation is ranking-only and now increases by one numeric
  // point per 0.2 m. It must not recreate the former nonlinear stop-like
  // score for an otherwise hard-safe exact Cartesian trajectory.
  EXPECT_LT(planner.selected()->total_cost, config.stop_cost);
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
  constexpr double kOpponentGapM = 10.0;
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

TEST(LatticePlanner,
     Runtime11Generation812CurrentPoseSoftBandCannotStopHardSafeExactPass) {
  sl::PlannerConfig config;
  config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  config.safety_evaluation_enabled = true;
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  config.exact_cartesian_execution_enabled = true;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};
  config.tangent_scales = {0.8, 1.0, 1.2};
  config.hard_max_steer_rad = 0.64;
  config.planner_max_steer_rad = 0.64;
  config.max_steer_rate_radps = 0.5;
  config.opponent_hard_clearance_m = 0.25;
  config.collision_max_step_m = 0.05;
  config.collision_max_yaw_step_rad = 0.0174533;
  config.opponent_lateral_tracking_margin_m = 0.0;
  config.opponent_longitudinal_tracking_margin_m = 0.10;
  config.horizon_points = 20;
  config.projection_initial_half_width_m = 3.0;
  config.projection_follow_half_width_m = 2.0;
  config.projection_max_backward_m = 0.05;

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
  ego.x = 89631.3713720122;
  ego.y = 43127.902675540194;
  ego.yaw = 2.1026863434939163;
  ego.speed_mps = 0.17542299891004245;
  ego.yaw_rate_radps = -0.008922820768399462;
  ego.stamp_sec = 46.499998960;
  ego.curvature = ego.yaw_rate_radps / ego.speed_mps;
  ego.frenet = planning_frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  ASSERT_TRUE(ego.valid);

  constexpr double previous_stamp = 46.399998962;
  constexpr double current_stamp = 46.449998961;
  const auto make_opponent = [&planning_frame](
                                 const std::string &id, double x, double y,
                                 double previous_x, double previous_y) {
    sl::OpponentState opponent;
    opponent.id = id;
    opponent.x = x;
    opponent.y = y;
    opponent.stamp_sec = current_stamp;
    const double dt = current_stamp - previous_stamp;
    opponent.vx_mps = (x - previous_x) / dt;
    opponent.vy_mps = (y - previous_y) / dt;
    opponent.speed_mps = std::hypot(opponent.vx_mps, opponent.vy_mps);
    opponent.sigma_x_m = 0.004999999888241291;
    opponent.sigma_y_m = 0.004999999888241291;
    opponent.uncertainty_x_m = 0.15;
    opponent.uncertainty_y_m = 0.15;
    opponent.frenet = planning_frame.project(opponent.x, opponent.y, 0.0);
    opponent.yaw = opponent.speed_mps > 0.3
                       ? std::atan2(opponent.vy_mps, opponent.vx_mps)
                       : planning_frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet =
        planning_frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.valid = opponent.frenet.valid;
    return opponent;
  };
  const std::vector<sl::OpponentState> opponents = {
      make_opponent("d2", 89628.9140625, 43131.0, 89628.9140625,
                    43131.0),
      make_opponent("d3", 89624.7109375, 43137.746105407714844,
                    89624.7109375, 43137.746166442871094),
      make_opponent("d4", 89620.234375, 43144.671875, 89620.234375,
                    43144.671875),
  };
  ASSERT_TRUE(std::all_of(opponents.begin(), opponents.end(),
                          [](const auto &opponent) {
                            return opponent.valid;
                          }));

  constexpr double now_sec = 46.514998960;
  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto output = planner.update(ego, opponents, true, now_sec);
  ASSERT_EQ(output.mode, sl::BehaviorMode::OVERTAKE_RIGHT) << output.reason;
  EXPECT_EQ(output.reason, "selected_right_candidate");
  EXPECT_TRUE(output.safe_lateral);
  EXPECT_FALSE(output.emergency_stop);
  EXPECT_EQ(output.execution_geometry_kind,
            sl::ExecutionGeometryKind::EXACT_CARTESIAN);
  EXPECT_EQ(output.output_horizon_diagnostic.failure,
            sl::OutputHorizonFailure::NONE);
  EXPECT_EQ(output.minimum_cost, 417);
  const auto wire_output =
      sl::prepareWireOutputForPublication(output, true);
  const auto payload = sl::makeWirePayload(
      wire_output, 812U, sl::WirePublicationPolicy{true, true});
  EXPECT_EQ(payload.kind, sl::WireKind::SPATIAL_LATERAL_AND_SPEED_V4);

  ASSERT_TRUE(planner.selected().has_value());
  const auto &selected = planner.selected().value();
  EXPECT_TRUE(selected.feasible);
  EXPECT_EQ(selected.total_cost, 417);
  EXPECT_EQ(selected.reference_cost, 17);
  EXPECT_EQ(selected.object_cost, 370);
  EXPECT_EQ(selected.safety_cost, 346);
  EXPECT_LT(selected.safety_cost, config.stop_cost);
  EXPECT_GT(selected.dynamic_speed_limit_mps, config.safe_stop_speed_mps);
  EXPECT_GT(output.speed_cap_mps, config.safe_stop_speed_mps);
  EXPECT_LE(output.speed_cap_mps,
            selected.dynamic_speed_limit_mps + config.tie_break_epsilon);
  ASSERT_EQ(selected.representative_object_diagnostic_count, 5U);
  EXPECT_EQ(selected.representative_object_diagnostics[0].object_level, 5);
  EXPECT_EQ(selected.representative_object_diagnostics[1].object_level, 9);
  EXPECT_EQ(selected.representative_object_diagnostics[2].object_level, 7);
  EXPECT_EQ(selected.representative_object_diagnostics[3].object_level, 7);
  EXPECT_EQ(selected.representative_object_diagnostics[4].object_level, 7);

  sl::OpponentState future_collision = opponents.front();
  const auto &collision_point = selected.dense.at(38U);
  future_collision.id = "future_exact_path_collision";
  future_collision.x = collision_point.x;
  future_collision.y = collision_point.y;
  future_collision.yaw = collision_point.yaw;
  future_collision.vx_mps = 0.0;
  future_collision.vy_mps = 0.0;
  future_collision.speed_mps = 0.0;
  future_collision.frenet = planning_frame.project(
      future_collision.x, future_collision.y, future_collision.yaw);
  future_collision.valid = future_collision.frenet.valid;
  ASSERT_TRUE(future_collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner.boundedExactCartesianExecution(
      selected, {future_collision}, 256U, &hard_negative_diagnostic));
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);

  auto current_collision = opponents.front();
  current_collision.id = "current_pose_collision";
  current_collision.x = ego.x;
  current_collision.y = ego.y;
  current_collision.yaw = ego.yaw;
  current_collision.vx_mps = 0.0;
  current_collision.vy_mps = 0.0;
  current_collision.speed_mps = 0.0;
  current_collision.frenet = planning_frame.project(
      current_collision.x, current_collision.y, current_collision.yaw);
  current_collision.valid = current_collision.frenet.valid;
  ASSERT_TRUE(current_collision.valid);
  sl::LatticePlanner hard_collision_planner(
      config, &planning_frame, &map, &output_frame);
  const auto hard_stop = hard_collision_planner.update(
      ego, {current_collision}, true, now_sec);
  EXPECT_EQ(hard_stop.mode, sl::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(hard_stop.reason, "current_pose_hard_collision");
  EXPECT_TRUE(hard_stop.emergency_stop);
  EXPECT_FALSE(hard_stop.safe_lateral);
}

namespace {
struct CampaignTrackabilityEgoInput {
  double stamp_sec;
  double x_m;
  double y_m;
  double quaternion_z;
  double quaternion_w;
  double speed_mps;
  double yaw_rate_radps;
};

struct CampaignTrackabilityOpponentInput {
  const char *id;
  double previous_stamp_sec;
  double previous_x_m;
  double previous_y_m;
  double stamp_sec;
  double x_m;
  double y_m;
};

constexpr CampaignTrackabilityEgoInput kCampaignEgo599{
    35.809999199,         89631.35696857203,  43128.068511178426,
    0.8572555657080112,   0.5148911487514984, 0.15838630163292242,
    -0.010774439149173125};
constexpr CampaignTrackabilityEgoInput kCampaignEgo600{
    35.844999198,         89631.35399294275,  43128.074104351115,
    0.8571445925411222,   0.5150758657494191, 0.15838630163292242,
    -0.010774439149173125};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignOpponents599{{
        {"d2", 35.749999200, 89628.9140625, 43131.0, 35.799999199,
         89628.9140625, 43131.0},
        {"d3", 35.749999200, 89624.7265625, 43137.74609375, 35.799999199,
         89624.7265625, 43137.74609375},
        {"d4", 35.749999200, 89620.234375, 43144.671875, 35.799999199,
         89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignOpponents600{{
        {"d2", 35.799999199, 89628.9140625, 43131.0, 35.849999198,
         89628.9140625, 43131.0},
        {"d3", 35.799999199, 89624.7265625, 43137.74609375, 35.849999198,
         89624.7265625, 43137.74609375},
        {"d4", 35.799999199, 89620.234375, 43144.671875, 35.849999198,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV2Ego482{
    29.609999338,        89631.37246477004, 43128.115957525646,
    0.8561840104693411,  0.5166710173956298, 0.25362589674154506,
    -0.006169710439701915};
constexpr CampaignTrackabilityEgoInput kCampaignV2Ego483{
    29.644999337,        89631.36773670123, 43128.12493344413,
    0.8561202497415139,  0.5167766616078243, 0.25362589674154506,
    -0.006169710439701915};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV2Opponents{{
        {"d2", 29.549999339, 89628.9140625, 43131.0, 29.599999338,
         89628.9140625, 43131.0},
        {"d3", 29.549999339, 89624.7265625, 43137.74609375, 29.599999338,
         89624.7265625, 43137.74609375},
        {"d4", 29.549999339, 89620.234375, 43144.671875, 29.599999338,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV3Ego902{
    52.839998818,        89631.33679129393, 43127.998389687506,
    0.8692451391799942, 0.4943813184293601, 0.052320205885175226,
    0.0014956074941825253};
constexpr CampaignTrackabilityEgoInput kCampaignV3Ego903{
    52.899998817,         89631.33501262346, 43128.00130202824,
    0.8692433487261981,  0.4943844664785344, 0.055119864779560636,
    -0.0005004838071090247};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV3Opponents{{
        {"d2", 52.799998819, 89628.9140625, 43131.0, 52.849998818,
         89628.9140625, 43131.0},
        {"d3", 52.799998819, 89624.703125, 43137.74609375, 52.849998818,
         89624.703125, 43137.74609375},
        {"d4", 52.799998819, 89620.234375, 43144.671875, 52.849998818,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV4Ego840{
    47.324998942,        89631.25454283068, 43128.124174137796,
    0.8654107597271219, 0.501063086794992, 0.19200096765060023,
    -0.004615206361117283};
constexpr CampaignTrackabilityEgoInput kCampaignV4Ego841{
    47.384998940,         89631.24864022767, 43128.1343175577,
    0.8653649368446305,  0.5011422214101391, 0.19280444316818304,
    -0.004210779563276817};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV4Opponents840{{
        {"d2", 47.249998943, 89628.9140625, 43131.0, 47.299998942,
         89628.9140625, 43131.0},
        {"d3", 47.249998943, 89624.7109375, 43137.74609375,
         47.299998942, 89624.7109375, 43137.74609375},
        {"d4", 47.249998943, 89620.234375, 43144.671875, 47.299998942,
         89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV4Opponents841{{
        {"d2", 47.299998942, 89628.9140625, 43131.0, 47.349998941,
         89628.9140625, 43131.0},
        {"d3", 47.299998942, 89624.7109375, 43137.74609375,
         47.349998941, 89624.7109375, 43137.74609375},
        {"d4", 47.299998942, 89620.234375, 43144.671875, 47.349998941,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV8Ego571{
    34.144999236, 89631.20621123779, 43128.41851431184,
    0.8358061015905128, 0.5490247358216835, 0.4684578136162057,
    -0.047268504973395215};
constexpr CampaignTrackabilityEgoInput kCampaignV8Ego572{
    34.214999235, 89631.19419283069, 43128.445049728834,
    0.8356496054958418, 0.5492629032026868, 0.47857857755652755,
    -0.018459290095729906};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV8Opponents571{{
        {"d2", 34.099999237, 89628.9140625, 43131.0, 34.149999236,
         89628.9140625, 43131.0},
        {"d3", 34.099999237, 89624.7265625, 43137.74609375, 34.149999236,
         89624.7265625, 43137.74609375},
        {"d4", 34.099999237, 89620.234375, 43144.671875, 34.149999236,
         89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV8Opponents572{{
        {"d2", 34.149999236, 89628.9140625, 43131.0, 34.199999235,
         89628.9140625, 43131.0},
        {"d3", 34.149999236, 89624.7265625, 43137.74609375, 34.199999235,
         89624.7265625, 43137.74609375},
        {"d4", 34.149999236, 89620.234375, 43144.671875, 34.199999235,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV9Ego731{
    41.974999061, 89631.44939406535, 43128.616214158275,
    0.8163265853427629, 0.5775906042021673, 0.0977019870875692,
    -0.009243317917513446};
constexpr CampaignTrackabilityEgoInput kCampaignV9Ego732{
    42.019999060, 89631.44803306558, 43128.62006601087,
    0.8162455647005820, 0.5777050961404340, 0.09933786929558507,
    -0.007121075401136209};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV9Opponents{{
        {"d2", 41.899999063, 89628.9140625, 43131.0, 41.949999062,
         89628.9140625, 43131.0},
        {"d3", 41.899999063, 89624.71875, 43137.74609375, 41.949999062,
         89624.71875, 43137.74609375},
        {"d4", 41.899999063, 89620.234375, 43144.671875, 41.949999062,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV10Ego590{
    37.709999157, 89631.27974632231, 43128.78216111484,
    0.8252617781532579, 0.5647503851431382, 0.1538064906440742,
    -0.005986345228332437};
constexpr CampaignTrackabilityEgoInput kCampaignV10Ego591{
    37.824999154, 89631.27308828691, 43128.79930750506,
    0.8250425552346151, 0.5650705991749501, 0.15262001193431204,
    -0.0071900550707856255};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV10Opponents590{{
        {"d2", 37.699999157, 89628.9140625, 43131.0, 37.649999157,
         89628.9140625, 43131.0},
        {"d3", 37.699999157, 89624.71875, 43137.74609375, 37.649999157,
         89624.71875, 43137.74609375},
        {"d4", 37.699999157, 89620.234375, 43144.671875, 37.649999157,
         89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV10Opponents591{{
        {"d2", 37.799999155, 89628.9140625, 43131.0, 37.749999156,
         89628.9140625, 43131.0},
        {"d3", 37.799999155, 89624.71875, 43137.74609375, 37.749999156,
         89624.71875, 43137.74609375},
        {"d4", 37.799999155, 89620.234375, 43144.671875, 37.749999156,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV11Ego557{
    34.069999238, 89631.23455423902, 43128.469965651915,
    0.8368654854595201, 0.5474085852876275, 0.18326131839530427,
    -0.00778657147654793};
constexpr CampaignTrackabilityEgoInput kCampaignV11Ego558{
    34.084999238, 89631.233085627973, 43128.473323786202,
    0.836822858562, 0.547473746757, 0.183261318395,
    -0.007786571477};
constexpr CampaignTrackabilityEgoInput kCampaignV11Ego559{
    34.124999237, 89631.230045131582, 43128.479978878662,
    0.836751194944, 0.547583270160, 0.182543040896,
    -0.008755584452};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV11Opponents557And558{{
        {"d2", 33.999999239, 89628.9140625, 43131.0, 34.049999238,
         89628.9140625, 43131.0},
        {"d3", 33.999999239, 89624.7265625, 43137.74609375,
         34.049999238, 89624.7265625, 43137.74609375},
        {"d4", 33.999999239, 89620.234375, 43144.671875,
         34.049999238, 89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV11Opponents559{{
        {"d2", 34.049999238, 89628.9140625, 43131.0, 34.099999237,
         89628.9140625, 43131.0},
        {"d3", 34.049999238, 89624.7265625, 43137.74609375,
         34.099999237, 89624.7265625, 43137.74609375},
        {"d4", 34.049999238, 89620.234375, 43144.671875,
         34.099999237, 89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV12Ego701{
    41.294999076, 89631.4051938334, 43128.641595683825,
    0.8164579341011834, 0.5774049201758049, 0.15117416247464846,
    -0.008354876710490502};
constexpr CampaignTrackabilityEgoInput kCampaignV12Ego702{
    41.374999075, 89631.40105227826, 43128.65320185036,
    0.8163712168402932, 0.577527519963074, 0.1515850192132884,
    -0.004097812232910528};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV12Opponents701{{
        {"d2", 41.199999079, 89628.9140625, 43131.0, 41.249999077,
         89628.9140625, 43131.0},
        {"d3", 41.199999079, 89624.71875, 43137.74609375, 41.249999077,
         89624.71875, 43137.74609375},
        {"d4", 41.199999079, 89620.234375, 43144.671875, 41.249999077,
         89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV12Opponents702{{
        {"d2", 41.299999076, 89628.9140625, 43131.0, 41.349999075,
         89628.9140625, 43131.0},
        {"d3", 41.299999076, 89624.71875, 43137.74609375, 41.349999075,
         89624.71875, 43137.74609375},
        {"d4", 41.299999076, 89620.234375, 43144.671875, 41.349999075,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV16Ego701{
    43.334999031, 89631.22177747132, 43128.78490166712,
    0.8174177521655053, 0.5760453267275871, 0.04298023937091172,
    -0.004549024714772127};
constexpr CampaignTrackabilityEgoInput kCampaignV16Ego702{
    43.474999028, 89631.2194331941, 43128.79145403971,
    0.817148109646438, 0.5764277638188959, 0.05384848386180184,
    -0.00734367215593885};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV16Opponents701{{
        {"d2", 43.299999032, 89628.9140625, 43131.0, 43.349999031,
         89628.9140625, 43131.0},
        {"d3", 43.299999032, 89624.7109375, 43137.74609375,
         43.349999031, 89624.7109375, 43137.74609375},
        {"d4", 43.299999032, 89620.234375, 43144.671875, 43.349999031,
         89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV16Opponents702{{
        {"d2", 43.399999029, 89628.9140625, 43131.0, 43.449999028,
         89628.9140625, 43131.0},
        {"d3", 43.399999029, 89624.7109375, 43137.74609375,
         43.449999028, 89624.7109375, 43137.74609375},
        {"d4", 43.399999029, 89620.234375, 43144.671875, 43.449999028,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV13PlannerInput597{
    36.684999180, 89631.475901656, 43128.68940659293,
    0.8038008707152043, 0.594898445314391, 0.14059643799759025,
    -0.015829784273565257};
constexpr CampaignTrackabilityEgoInput kCampaignV13PlannerInput598{
    36.804999177, 89631.47092550521, 43128.70552022181,
    0.8032334194265623, 0.5956643970528306, 0.14024926542199614,
    -0.016325020779173513};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV13Opponents597{{
        {"d2", 36.599999181, 89628.9140625, 43131.0, 36.649999180,
         89628.9140625, 43131.0},
        {"d3", 36.599999181, 89624.7265625, 43137.74609375,
         36.649999180, 89624.7265625, 43137.74609375},
        {"d4", 36.599999181, 89620.234375, 43144.671875, 36.649999180,
         89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV13Opponents598{{
        {"d2", 36.699999179, 89628.9140625, 43131.0, 36.749999178,
         89628.9140625, 43131.0},
        {"d3", 36.699999179, 89624.7265625, 43137.74609375,
         36.749999178, 89624.7265625, 43137.74609375},
        {"d4", 36.699999179, 89620.234375, 43144.671875, 36.749999178,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV14PlannerInput613{
    39.049999127, 89631.29687071475, 43128.90516422057,
    0.7905274934481581, 0.612426552414714, 0.0907454066832481,
    -0.01056415573609255};
constexpr CampaignTrackabilityEgoInput kCampaignV14PlannerInput614{
    39.109999125, 89631.29543419414, 43128.91067323464,
    0.7903362399089036, 0.6126733451739648, 0.0942499119579963,
    -0.010142039380082016};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV14Opponents613{{
        {"d2", 38.949999129, 89628.9140625, 43131.0, 38.999999128,
         89628.9140625, 43131.0},
        {"d3", 38.949999129, 89624.71875, 43137.74609375, 38.999999128,
         89624.71875, 43137.74609375},
        {"d4", 38.949999129, 89620.234375, 43144.671875, 38.999999128,
         89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV14Opponents614{{
        {"d2", 38.999999128, 89628.9140625, 43131.0, 39.049999127,
         89628.9140625, 43131.0},
        {"d3", 38.999999128, 89624.71875, 43137.74609375, 39.049999127,
         89624.71875, 43137.74609375},
        {"d4", 38.999999128, 89620.234375, 43144.671875, 39.049999127,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV15Ego533{
    34.304999233, 89631.24709566448, 43128.885107290014,
    0.7871600447017177, 0.6167487851833919, 0.150728929108568,
    -0.01709081340195702};
constexpr CampaignTrackabilityEgoInput kCampaignV15Ego534{
    34.424999230, 89631.2426464475, 43128.90237846602,
    0.7865680389518043, 0.6175036195031677, 0.14931425541460355,
    -0.016735911675582926};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV15Opponents533{{
        {"d2", 34.199999235, 89628.9140625, 43131.0, 34.249999234,
         89628.9140625, 43131.0},
        {"d3", 34.199999235, 89624.7265625, 43137.74609375,
         34.249999234, 89624.7265625, 43137.74609375},
        {"d4", 34.199999235, 89620.234375, 43144.671875, 34.249999234,
         89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV15Opponents534{{
        {"d2", 34.349999232, 89628.9140625, 43131.0, 34.399999231,
         89628.9140625, 43131.0},
        {"d3", 34.349999232, 89624.7265625, 43137.74609375,
         34.399999231, 89624.7265625, 43137.74609375},
        {"d4", 34.349999232, 89620.234375, 43144.671875, 34.399999231,
         89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV19Ego444{
    28.109999371, 89631.17348070661, 43128.31792346497,
    0.8575508942115735, 0.5143991289231842, 0.13652796180497112,
    -0.002842439164062367};
constexpr CampaignTrackabilityEgoInput kCampaignV19Ego445{
    28.164999370, 89631.16955547011, 43128.32516755075,
    0.8575158722692402, 0.514457509233099, 0.1376168892253623,
    -0.002604838322348142};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV19Opponents444{{
        {"d2", 27.999999374, 89628.9140625, 43131.0, 28.049999373,
         89628.9140625, 43131.0},
        {"d3", 27.999999374, 89624.734375, 43137.74609375,
         28.049999373, 89624.734375, 43137.74609375},
        {"d4", 27.999999374, 89620.234375, 43144.671875,
         28.049999373, 89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV19Opponents445{{
        {"d2", 28.049999373, 89628.9140625, 43131.0, 28.099999371,
         89628.9140625, 43131.0},
        {"d3", 28.049999373, 89624.734375, 43137.74609375,
         28.099999371, 89624.734375, 43137.74609375},
        {"d4", 28.049999373, 89620.234375, 43144.671875,
         28.099999371, 89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV21Ego499{
    30.444999319, 89631.12866477095, 43128.72367732412,
    0.8182969167640375, 0.5747957515626485, 0.162509790472036,
    -0.03115647398722412};
constexpr CampaignTrackabilityEgoInput kCampaignV21Ego500{
    30.544999317, 89631.12290464556, 43128.738730014644,
    0.8174810987311134, 0.5759554264154229, 0.16082905108979734,
    -0.03027668705173864};
constexpr CampaignTrackabilityEgoInput kCampaignV21Ego501 =
    kCampaignV21Ego500;
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV21Opponents499{{
        {"d2", 30.349999321, 89628.9140625, 43131.0, 30.399999320,
         89628.9140625, 43131.0},
        {"d3", 30.349999321, 89624.7265625, 43137.74609375,
         30.399999320, 89624.7265625, 43137.74609375},
        {"d4", 30.349999321, 89620.234375, 43144.671875,
         30.399999320, 89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV21Opponents500And501{{
        {"d2", 30.449999319, 89628.9140625, 43131.0, 30.499999318,
         89628.9140625, 43131.0},
        {"d3", 30.449999319, 89624.7265625, 43137.74609375,
         30.499999318, 89624.7265625, 43137.74609375},
        {"d4", 30.449999319, 89620.234375, 43144.671875,
         30.499999318, 89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV22Ego456{
    28.134999371, 89631.18576578503, 43128.84736829982,
    0.7887078905466942, 0.6147681379100448, 0.16162672606701323,
    -0.020673569590330277};
constexpr CampaignTrackabilityEgoInput kCampaignV22Ego457{
    28.234999368, 89631.18167361523, 43128.862527837446,
    0.7881266640332789, 0.6155130879516495, 0.15855496500273242,
    -0.019850581354642047};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV22Opponents456{{
        {"d2", 28.049999373, 89628.9140625, 43131.0, 28.099999371,
         89628.9140625, 43131.0},
        {"d3", 28.049999373, 89624.734375, 43137.75, 28.099999371,
         89624.734375, 43137.75},
        {"d4", 28.049999373, 89620.234375, 43144.671875,
         28.099999371, 89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV22Opponents457{{
        {"d2", 28.149999370, 89628.9140625, 43131.0, 28.199999369,
         89628.9140625, 43131.0},
        {"d3", 28.149999370, 89624.734375, 43137.75, 28.199999369,
         89624.734375, 43137.75},
        {"d4", 28.149999370, 89620.234375, 43144.671875,
         28.199999369, 89620.234375, 43144.671875},
    }};
constexpr CampaignTrackabilityEgoInput kCampaignV23Ego551{
    33.144999259, 89631.11361752331, 43128.75875317818,
    0.8066152495353758, 0.5910768471332501, 0.15211955982738723,
    -0.025720220740416317};
constexpr CampaignTrackabilityEgoInput kCampaignV23Ego552{
    33.244999256, 89631.11013552715, 43128.76869449146,
    0.8064265171789498, 0.5913343152486832, 0.09644412546095248,
    -0.006390543064612519};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV23Opponents551{{
        {"d2", 33.099999260, 89628.9140625, 43131.0, 33.149999259,
         89628.9140625, 43131.0},
        {"d3", 33.099999260, 89624.7265625, 43137.74609375,
         33.149999259, 89624.7265625, 43137.74609375},
        {"d4", 33.099999260, 89620.234375, 43144.671875,
         33.149999259, 89620.234375, 43144.671875},
    }};
constexpr std::array<CampaignTrackabilityOpponentInput, 3U>
    kCampaignV23Opponents552{{
        {"d2", 33.199999257, 89628.9140625, 43131.0, 33.249999256,
         89628.9140625, 43131.0},
        {"d3", 33.199999257, 89624.7265625, 43137.74609375,
         33.249999256, 89624.7265625, 43137.74609375},
        {"d4", 33.199999257, 89620.234375, 43144.671875,
         33.249999256, 89620.234375, 43144.671875},
    }};
sl::PlannerConfig campaignTrackabilityConfig(const sl::FrenetFrame &frame) {
  sl::PlannerConfig config;
  config.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  config.safety_evaluation_enabled = true;
  config.exact_cartesian_execution_enabled = true;
  config.experimental_exact_spatial_follow_shadow_enabled = true;
  config.reference_package = "state_lattice_overtake_planner";
  config.reference_csv = "data/course_centerline.csv";
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};
  config.tangent_scales = {0.8, 1.0, 1.2};
  config.hard_max_steer_rad = 0.64;
  config.planner_max_steer_rad = 0.64;
  config.max_steer_rate_radps = 0.5;
  config.planning_deadline_ms = 80.0;
  config.mpc_health_speed_guard_enabled = false;
  config.own_vehicle_id = "d1";
  config.overtake_permission_profile_enabled = true;
  config.default_overtake_allowed = true;
  const std::array<std::tuple<const char *, std::size_t, std::size_t>, 9U>
      permission_rows{{
          {"s1", 1U, 30U},
          {"s1_1", 30U, 55U},
          {"s2", 55U, 100U},
          {"s3", 100U, 155U},
          {"s4", 155U, 190U},
          {"s5", 190U, 265U},
          {"s6", 265U, 290U},
          {"s7", 290U, 320U},
          {"s8", 320U, 335U},
      }};
  for (const auto &[name, start, end] : permission_rows) {
    EXPECT_LT(start, frame.points().size());
    EXPECT_LT(end, frame.points().size());
    if (start < frame.points().size() && end < frame.points().size()) {
      config.overtake_permission_rules.push_back(sl::OvertakePermissionRule{
          name, frame.points()[start].s, frame.points()[end].s, true});
    }
  }
  return config;
}

sl::EgoState campaignTrackabilityEgo(const CampaignTrackabilityEgoInput &input,
                                     const sl::FrenetFrame &frame) {
  sl::EgoState ego;
  ego.stamp_sec = input.stamp_sec;
  ego.x = input.x_m;
  ego.y = input.y_m;
  ego.yaw = std::atan2(2.0 * input.quaternion_w * input.quaternion_z,
                       1.0 - 2.0 * input.quaternion_z * input.quaternion_z);
  ego.speed_mps = input.speed_mps;
  ego.yaw_rate_radps = input.yaw_rate_radps;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.curvature =
      std::abs(ego.speed_mps) > 0.1
          ? ego.yaw_rate_radps / ego.speed_mps
          : (ego.frenet.valid ? frame.interpolate(ego.frenet.s).kappa : 0.0);
  ego.valid = ego.frenet.valid && std::isfinite(ego.curvature);
  return ego;
}

template <std::size_t N>
std::vector<sl::OpponentState> campaignTrackabilityOpponents(
    const std::array<CampaignTrackabilityOpponentInput, N> &inputs,
    const sl::FrenetFrame &frame) {
  std::vector<sl::OpponentState> opponents;
  opponents.reserve(N);
  for (const auto &input : inputs) {
    sl::OpponentState opponent;
    opponent.id = input.id;
    opponent.stamp_sec = input.stamp_sec;
    opponent.x = input.x_m;
    opponent.y = input.y_m;
    const double dt = input.stamp_sec - input.previous_stamp_sec;
    opponent.vx_mps = (input.x_m - input.previous_x_m) / dt;
    opponent.vy_mps = (input.y_m - input.previous_y_m) / dt;
    opponent.speed_mps = std::hypot(opponent.vx_mps, opponent.vy_mps);
    opponent.sigma_x_m = 0.004999999888241291;
    opponent.sigma_y_m = 0.004999999888241291;
    opponent.uncertainty_x_m = 0.15;
    opponent.uncertainty_y_m = 0.15;
    opponent.frenet = frame.project(opponent.x, opponent.y, 0.0);
    opponent.yaw = opponent.speed_mps > 0.3
                       ? std::atan2(opponent.vy_mps, opponent.vx_mps)
                       : frame.interpolate(opponent.frenet.s).yaw;
    opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
    opponent.valid = opponent.frenet.valid;
    opponents.push_back(opponent);
  }
  return opponents;
}
} // namespace

TEST(LatticePlanner,
     CampaignGeneration600UsesBoundedFallbackOnlyAfterLegacyProfilesFail) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(
      planning_frame.loadCsv(std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
                                 "/data/course_centerline.csv",
                             &error))
      << error;
  ASSERT_TRUE(
      output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                               "/env/final_ver3/traj_mincurv_manual.csv",
                           &error))
      << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego599 = campaignTrackabilityEgo(kCampaignEgo599, planning_frame);
  const auto ego600 = campaignTrackabilityEgo(kCampaignEgo600, planning_frame);
  const auto opponents599 =
      campaignTrackabilityOpponents(kCampaignOpponents599, planning_frame);
  const auto opponents600 =
      campaignTrackabilityOpponents(kCampaignOpponents600, planning_frame);
  ASSERT_TRUE(ego599.valid);
  ASSERT_TRUE(ego600.valid);
  ASSERT_TRUE(std::all_of(opponents599.begin(), opponents599.end(),
                          [](const auto &opponent) { return opponent.valid; }));
  ASSERT_TRUE(std::all_of(opponents600.begin(), opponents600.end(),
                          [](const auto &opponent) { return opponent.valid; }));

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto generation599 = planner.update(ego599, opponents599, true, 35.810);
  ASSERT_EQ(generation599.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation599.reason;
  ASSERT_TRUE(planner.selected().has_value());
  const auto legacy_selected = planner.selected().value();
  ASSERT_EQ(legacy_selected.lateral_index, 2U);
  ASSERT_EQ(legacy_selected.tangent_index, 1U);
  EXPECT_NEAR(legacy_selected.goal_d_m, -1.0, 1.0e-12);
  // The legacy 0.50/0.80 profile remains first. If the new fallback ran
  // eagerly, this speed cap would instead be the 0.40/0.80 profile's value.
  EXPECT_NEAR(legacy_selected.dynamic_speed_limit_mps, 0.20001145296336637,
              1.0e-9);

  const auto generated600 = planner.generateCandidates(ego600, opponents600);
  const auto generated600_right = std::find_if(
      generated600.begin(), generated600.end(), [](const auto &candidate) {
        return candidate.lateral_index == 2U && candidate.tangent_index == 1U;
      });
  ASSERT_NE(generated600_right, generated600.end());
  EXPECT_TRUE(generated600_right->feasible)
      << generated600_right->rejection_reason
      << " dynamic_speed=" << generated600_right->dynamic_speed_limit_mps
      << " entry_speed=" << generated600_right->entry_speed_limit_mps
      << " dense=" << generated600_right->dense.size();

  const auto generation600 = planner.update(ego600, opponents600, true, 35.860);
  ASSERT_EQ(generation600.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation600.reason
      << " feasible=" << generation600.feasible_candidates
      << " trackability_rejects="
      << generation600.rejected_trackability_candidates;
  EXPECT_EQ(generation600.reason, "selected_right_candidate");
  EXPECT_TRUE(generation600.safe_lateral);
  EXPECT_FALSE(generation600.emergency_stop);
  ASSERT_TRUE(planner.selected().has_value());
  const auto fallback_selected = planner.selected().value();
  ASSERT_EQ(fallback_selected.lateral_index, 2U);
  ASSERT_EQ(fallback_selected.tangent_index, 1U);
  EXPECT_NEAR(fallback_selected.goal_d_m, -1.0, 1.0e-12);
  EXPECT_NEAR(fallback_selected.dynamic_speed_limit_mps, 0.20191305149184061,
              1.0e-9);
  EXPECT_LT(fallback_selected.clearance_profile_attempt_index, 60U);
  EXPECT_GT(fallback_selected.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  EXPECT_TRUE(planner.boundedExactCartesianExecution(
      fallback_selected, opponents600, 256U, &bounded_diagnostic))
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_EQ(bounded_diagnostic.failure, sl::OutputHorizonFailure::NONE);

  // A generic opponent_collision must not authorize recovery for the selected
  // blocker when another identity collides at the same point. The duplicated
  // pose makes the identity ambiguity deterministic without changing geometry.
  auto ambiguous_opponents = opponents600;
  auto third_party = opponents600.front();
  third_party.id = "third_party_same_pose";
  ambiguous_opponents.push_back(third_party);
  sl::LatticePlanner ambiguous_planner(config, &planning_frame, &map,
                                       &output_frame);
  const auto ambiguous_candidates =
      ambiguous_planner.generateCandidates(ego600, ambiguous_opponents);
  const auto ambiguous_right = std::find_if(
      ambiguous_candidates.begin(), ambiguous_candidates.end(),
      [](const auto &candidate) {
        return candidate.lateral_index == 2U && candidate.tangent_index == 1U;
      });
  ASSERT_NE(ambiguous_right, ambiguous_candidates.end());
  EXPECT_FALSE(ambiguous_right->feasible);
  EXPECT_FALSE(ambiguous_right->clearance_profile_applied);
  EXPECT_EQ(ambiguous_right->rejection_reason, "opponent_collision");
  EXPECT_TRUE(ambiguous_right->rejection_opponent_identity_ambiguous);

  // Missing identity on the selected blocker must never compare equal to the
  // empty "not recorded" sentinel and authorize clearance recovery.
  auto empty_blocker_id_opponents = opponents600;
  empty_blocker_id_opponents.front().id.clear();
  sl::LatticePlanner empty_blocker_id_planner(config, &planning_frame, &map,
                                               &output_frame);
  const auto empty_blocker_id_candidates =
      empty_blocker_id_planner.generateCandidates(ego600,
                                                   empty_blocker_id_opponents);
  const auto empty_blocker_id_right = std::find_if(
      empty_blocker_id_candidates.begin(), empty_blocker_id_candidates.end(),
      [](const auto &candidate) {
        return candidate.lateral_index == 2U && candidate.tangent_index == 1U;
      });
  ASSERT_NE(empty_blocker_id_right, empty_blocker_id_candidates.end());
  EXPECT_FALSE(empty_blocker_id_right->feasible);
  EXPECT_FALSE(empty_blocker_id_right->clearance_profile_applied);
  EXPECT_EQ(empty_blocker_id_right->rejection_reason, "opponent_collision");
  EXPECT_TRUE(
      empty_blocker_id_right->rejection_opponent_identity_ambiguous);

  // An empty-ID third party at the same collision pose must make the identity
  // ambiguous regardless of vector order, rather than disappearing into the
  // sentinel and authorizing recovery for the named selected blocker.
  auto empty_third_party_opponents = opponents600;
  auto empty_third_party = opponents600.front();
  empty_third_party.id.clear();
  empty_third_party_opponents.insert(empty_third_party_opponents.begin(),
                                     empty_third_party);
  sl::LatticePlanner empty_third_party_planner(config, &planning_frame, &map,
                                                &output_frame);
  const auto empty_third_party_candidates =
      empty_third_party_planner.generateCandidates(ego600,
                                                    empty_third_party_opponents);
  const auto empty_third_party_right = std::find_if(
      empty_third_party_candidates.begin(), empty_third_party_candidates.end(),
      [](const auto &candidate) {
        return candidate.lateral_index == 2U && candidate.tangent_index == 1U;
      });
  ASSERT_NE(empty_third_party_right, empty_third_party_candidates.end());
  EXPECT_FALSE(empty_third_party_right->feasible);
  EXPECT_FALSE(empty_third_party_right->clearance_profile_applied);
  EXPECT_EQ(empty_third_party_right->rejection_reason,
            "opponent_collision");
  EXPECT_TRUE(
      empty_third_party_right->rejection_opponent_identity_ambiguous);

  sl::OpponentState collision = opponents600.front();
  const auto &collision_point =
      fallback_selected.dense.at(fallback_selected.dense.size() / 2U);
  collision.id = "fallback_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner.boundedExactCartesianExecution(
      fallback_selected, {collision}, 256U, &hard_negative_diagnostic));
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);

}

TEST(LatticePlanner,
     CampaignV14AcceptedProfileContinuityNeighbourhoodClosesGen614) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) + "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego613 =
      campaignTrackabilityEgo(kCampaignV14PlannerInput613, planning_frame);
  const auto ego614 =
      campaignTrackabilityEgo(kCampaignV14PlannerInput614, planning_frame);
  const auto opponents613 = campaignTrackabilityOpponents(
      kCampaignV14Opponents613, planning_frame);
  const auto opponents614 = campaignTrackabilityOpponents(
      kCampaignV14Opponents614, planning_frame);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      planner, "d2", -1, 2U, 1U, -1.0, 8.849, 38.999999128, 612U,
      true, 0.31, 0.69);
  const auto generation613 = planner.update(ego613, opponents613, true, 39.090);
  ASSERT_EQ(generation613.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation613.reason;
  ASSERT_TRUE(planner.selected().has_value());
  EXPECT_TRUE(planner.selected()->clearance_profile_applied);
  EXPECT_NEAR(planner.selected()->clearance_profile_join_fraction, 0.31,
              1.0e-12);
  EXPECT_NEAR(planner.selected()->clearance_profile_yaw_magnitude, 0.69,
              1.0e-12);

  const auto generation614_start = std::chrono::steady_clock::now();
  const auto generation614 = planner.update(ego614, opponents614, true, 39.140);
  const double generation614_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - generation614_start)
          .count();
  ASSERT_EQ(generation614.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation614.reason;
  EXPECT_EQ(generation614.feasible_candidates, 1);
  EXPECT_GT(generation614.rejected_opponent_candidates, 0);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected614 = planner.selected().value();
  EXPECT_TRUE(selected614.clearance_profile_applied);
  EXPECT_NEAR(selected614.clearance_profile_join_fraction, 0.31, 1.0e-12);
  EXPECT_NEAR(selected614.clearance_profile_yaw_magnitude, 0.68, 1.0e-12);
  EXPECT_GT(selected614.dynamic_speed_limit_mps, config.safe_stop_speed_mps);
  EXPECT_LT(generation614_ms, config.planning_deadline_ms);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded = planner.boundedExactCartesianExecution(
      selected614, opponents614, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded.has_value()) << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded->dense.size(), 256U);

  sl::OpponentState collision = opponents614.front();
  const auto &collision_point =
      selected614.dense.at(selected614.dense.size() / 2U);
  collision.id = "v14_gen614_continuity_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic collision_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected614, {collision}, 256U, &collision_diagnostic)
                   .has_value());

  sl::LatticePlanner invalid_hint_planner(config, &planning_frame, &map,
                                          &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      invalid_hint_planner, "d2", -1, 2U, 1U, -1.0, 8.844,
      39.049999127, 613U, true,
      std::numeric_limits<double>::quiet_NaN(), 0.69);
  const auto invalid_hint =
      invalid_hint_planner.update(ego614, opponents614, true, 39.140);
  EXPECT_EQ(invalid_hint.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(invalid_hint.feasible_candidates, 0);
  EXPECT_FALSE(invalid_hint_planner.selected().has_value());
}

TEST(LatticePlanner, CampaignV15AcceptedProfileOuterRingClosesGen534) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) + "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  auto config = campaignTrackabilityConfig(planning_frame);
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));
  const auto ego533 = campaignTrackabilityEgo(kCampaignV15Ego533, planning_frame);
  const auto ego534 = campaignTrackabilityEgo(kCampaignV15Ego534, planning_frame);
  const auto opponents533 =
      campaignTrackabilityOpponents(kCampaignV15Opponents533, planning_frame);
  const auto opponents534 =
      campaignTrackabilityOpponents(kCampaignV15Opponents534, planning_frame);
  ASSERT_TRUE(ego533.valid);
  ASSERT_FALSE(opponents533.empty());

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      planner, "d2", -1, 2U, 1U, -1.0, 9.008, 34.249999234, 532U,
      true, 0.31, 0.69);
  const auto result533 = planner.update(ego533, opponents533, true, 34.355);
  ASSERT_EQ(result533.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << result533.reason;
  ASSERT_TRUE(planner.selected().has_value());
  EXPECT_EQ(planner.selected()->lateral_index, 2U);
  EXPECT_EQ(planner.selected()->tangent_index, 1U);
  EXPECT_TRUE(planner.selected()->clearance_profile_applied);
  const auto retained_hint =
      sl::StateLatticeTestAccess::clearanceProfileHint(planner);
  ASSERT_TRUE(retained_hint.has_value());
  EXPECT_NEAR(retained_hint->first, 0.30, 1.0e-12);
  EXPECT_NEAR(retained_hint->second, 0.69, 1.0e-12);
  const auto start = std::chrono::steady_clock::now();
  const auto result534 = planner.update(ego534, opponents534, true, 34.475);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();
  ASSERT_EQ(result534.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << result534.reason;
  EXPECT_EQ(result534.feasible_candidates, 1);
  EXPECT_GT(result534.rejected_opponent_candidates, 0);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected = planner.selected().value();
  EXPECT_TRUE(selected.clearance_profile_applied);
  EXPECT_NEAR(selected.clearance_profile_join_fraction, 0.28, 1.0e-12);
  EXPECT_NEAR(selected.clearance_profile_yaw_magnitude, 0.68, 1.0e-12);
  EXPECT_GT(selected.dynamic_speed_limit_mps, config.safe_stop_speed_mps);
  EXPECT_LT(elapsed_ms, config.planning_deadline_ms);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded = planner.boundedExactCartesianExecution(
      selected, opponents534, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded.has_value()) << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded->dense.size(), 256U);

  sl::OpponentState collision = opponents534.front();
  const auto &collision_point = selected.dense.at(selected.dense.size() / 2U);
  collision.id = "v15_gen534_outer_ring_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet = planning_frame.project(collision.x, collision.y,
                                             collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic collision_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected, {collision}, 256U, &collision_diagnostic)
                   .has_value());

  sl::LatticePlanner invalid_hint_planner(config, &planning_frame, &map,
                                          &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      invalid_hint_planner, "d2", -1, 2U, 1U, -1.0, 9.008,
      34.399999231, 533U, true,
      std::numeric_limits<double>::quiet_NaN(), 0.69);
  const auto invalid_hint =
      invalid_hint_planner.update(ego534, opponents534, true, 34.475);
  EXPECT_EQ(invalid_hint.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(invalid_hint.feasible_candidates, 0);
  EXPECT_FALSE(invalid_hint_planner.selected().has_value());
}

TEST(LatticePlanner,
     CampaignV2Generation483UsesFinalBoundedFallbackAfterExistingProfilesFail) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(
      planning_frame.loadCsv(std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
                                 "/data/course_centerline.csv",
                             &error))
      << error;
  ASSERT_TRUE(
      output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                               "/env/final_ver3/traj_mincurv_manual.csv",
                           &error))
      << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego482 =
      campaignTrackabilityEgo(kCampaignV2Ego482, planning_frame);
  const auto ego483 =
      campaignTrackabilityEgo(kCampaignV2Ego483, planning_frame);
  const auto opponents =
      campaignTrackabilityOpponents(kCampaignV2Opponents, planning_frame);
  ASSERT_TRUE(ego482.valid);
  ASSERT_TRUE(ego483.valid);
  ASSERT_TRUE(std::all_of(opponents.begin(), opponents.end(),
                          [](const auto &opponent) { return opponent.valid; }));

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto generation482 = planner.update(ego482, opponents, true, 29.615);
  ASSERT_EQ(generation482.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation482.reason;
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected482 = planner.selected().value();
  EXPECT_EQ(selected482.lateral_index, 2U);
  EXPECT_EQ(selected482.tangent_index, 1U);
  EXPECT_NEAR(selected482.goal_d_m, -1.0, 1.0e-12);
  // The existing (0.40, 0.80) profile remains authoritative when it succeeds.
  EXPECT_NEAR(selected482.dynamic_speed_limit_mps, 0.20029745142099847,
              1.0e-9);

  const auto generated483 = planner.generateCandidates(ego483, opponents);
  const auto generated483_right = std::find_if(
      generated483.cbegin(), generated483.cend(), [](const auto &candidate) {
        return candidate.lateral_index == 2U && candidate.tangent_index == 1U;
      });
  ASSERT_NE(generated483_right, generated483.cend());
  ASSERT_TRUE(generated483_right->feasible)
      << generated483_right->rejection_reason
      << " dynamic_speed_limit_mps="
      << generated483_right->dynamic_speed_limit_mps
      << " dense_points=" << generated483_right->dense.size();
  EXPECT_NEAR(generated483_right->dynamic_speed_limit_mps,
              0.20158430350515344, 1.0e-9);

  const auto generation483 = planner.update(ego483, opponents, true, 29.665);
  ASSERT_EQ(generation483.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation483.reason
      << " feasible=" << generation483.feasible_candidates
      << " trackability_rejects="
      << generation483.rejected_trackability_candidates;
  EXPECT_EQ(generation483.reason, "selected_right_candidate");
  EXPECT_TRUE(generation483.safe_lateral);
  EXPECT_FALSE(generation483.emergency_stop);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected483 = planner.selected().value();
  EXPECT_EQ(selected483.lateral_index, 2U);
  EXPECT_EQ(selected483.tangent_index, 1U);
  EXPECT_NEAR(selected483.goal_d_m, -1.0, 1.0e-12);
  EXPECT_NEAR(selected483.dynamic_speed_limit_mps, 0.20158430350515344,
              1.0e-9);
  EXPECT_LT(selected483.clearance_profile_attempt_index, 60U);
  EXPECT_GT(selected483.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  EXPECT_TRUE(planner.boundedExactCartesianExecution(
      selected483, opponents, 256U, &bounded_diagnostic))
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_EQ(bounded_diagnostic.failure, sl::OutputHorizonFailure::NONE);

  sl::OpponentState collision = opponents.front();
  const auto &collision_point =
      selected483.dense.at(selected483.dense.size() / 2U);
  collision.id = "v2_fallback_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner.boundedExactCartesianExecution(
      selected483, {collision}, 256U, &hard_negative_diagnostic));
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);

}

TEST(LatticePlanner,
     CampaignV3Generation903UsesFinalGridCompletionAfterExistingProfilesFail) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(
      planning_frame.loadCsv(std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
                                 "/data/course_centerline.csv",
                             &error))
      << error;
  ASSERT_TRUE(
      output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                               "/env/final_ver3/traj_mincurv_manual.csv",
                           &error))
      << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego902 =
      campaignTrackabilityEgo(kCampaignV3Ego902, planning_frame);
  const auto ego903 =
      campaignTrackabilityEgo(kCampaignV3Ego903, planning_frame);
  const auto opponents =
      campaignTrackabilityOpponents(kCampaignV3Opponents, planning_frame);
  ASSERT_TRUE(ego902.valid);
  ASSERT_TRUE(ego903.valid);
  ASSERT_TRUE(std::all_of(opponents.begin(), opponents.end(),
                          [](const auto &opponent) { return opponent.valid; }));

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto generation902 = planner.update(ego902, opponents, true, 52.850);
  ASSERT_EQ(generation902.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation902.reason;
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected902 = planner.selected().value();
  EXPECT_EQ(selected902.lateral_index, 2U);
  EXPECT_EQ(selected902.tangent_index, 1U);
  EXPECT_NEAR(selected902.goal_d_m, -1.0, 1.0e-12);
  // Existing (0.50, 0.80) remains first; the final grid completion must not
  // replace a profile that already passes the unchanged evaluator.
  EXPECT_NEAR(selected902.dynamic_speed_limit_mps, 0.2004931701099154,
              1.0e-9);

  const auto generated903 = planner.generateCandidates(ego903, opponents);
  const auto generated903_right = std::find_if(
      generated903.cbegin(), generated903.cend(), [](const auto &candidate) {
        return candidate.lateral_index == 2U && candidate.tangent_index == 1U;
      });
  ASSERT_NE(generated903_right, generated903.cend());
  ASSERT_TRUE(generated903_right->feasible)
      << generated903_right->rejection_reason
      << " dynamic_speed_limit_mps="
      << generated903_right->dynamic_speed_limit_mps
      << " dense_points=" << generated903_right->dense.size();
  EXPECT_NEAR(generated903_right->dynamic_speed_limit_mps,
              0.20280435742374198, 1.0e-9);

  const auto generation903 = planner.update(ego903, opponents, true, 52.900);
  ASSERT_EQ(generation903.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation903.reason
      << " feasible=" << generation903.feasible_candidates
      << " trackability_rejects="
      << generation903.rejected_trackability_candidates;
  EXPECT_EQ(generation903.reason, "selected_right_candidate");
  EXPECT_TRUE(generation903.safe_lateral);
  EXPECT_FALSE(generation903.emergency_stop);
  EXPECT_TRUE(generation903.pass_continuation_active);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected903 = planner.selected().value();
  EXPECT_EQ(selected903.lateral_index, 2U);
  EXPECT_EQ(selected903.tangent_index, 1U);
  EXPECT_NEAR(selected903.goal_d_m, -1.0, 1.0e-12);
  EXPECT_NEAR(selected903.dynamic_speed_limit_mps, 0.20280435742374198,
              1.0e-9);
  EXPECT_LT(selected903.clearance_profile_attempt_index, 60U);
  EXPECT_GT(selected903.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded_execution = planner.boundedExactCartesianExecution(
      selected903, opponents, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded_execution.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_EQ(bounded_diagnostic.failure, sl::OutputHorizonFailure::NONE);
  EXPECT_LE(bounded_execution->dense.size(), 256U);

  sl::OpponentState collision = opponents.front();
  const auto &collision_point =
      selected903.dense.at(selected903.dense.size() / 2U);
  collision.id = "v3_grid_completion_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected903, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);
}

TEST(LatticePlanner,
     CampaignV4Generation841UsesBoundedRobustRegionAfterExistingProfilesFail) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(
      planning_frame.loadCsv(std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
                                 "/data/course_centerline.csv",
                             &error))
      << error;
  ASSERT_TRUE(
      output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                               "/env/final_ver3/traj_mincurv_manual.csv",
                           &error))
      << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error))
      << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego840 =
      campaignTrackabilityEgo(kCampaignV4Ego840, planning_frame);
  const auto ego841 =
      campaignTrackabilityEgo(kCampaignV4Ego841, planning_frame);
  const auto opponents840 = campaignTrackabilityOpponents(
      kCampaignV4Opponents840, planning_frame);
  const auto opponents841 = campaignTrackabilityOpponents(
      kCampaignV4Opponents841, planning_frame);
  ASSERT_TRUE(ego840.valid);
  ASSERT_TRUE(ego841.valid);
  ASSERT_TRUE(std::all_of(opponents840.begin(), opponents840.end(),
                          [](const auto &opponent) { return opponent.valid; }));
  ASSERT_TRUE(std::all_of(opponents841.begin(), opponents841.end(),
                          [](const auto &opponent) { return opponent.valid; }));

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto generation840 =
      planner.update(ego840, opponents840, true, 47.345);
  ASSERT_EQ(generation840.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation840.reason;
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected840 = planner.selected().value();
  EXPECT_EQ(selected840.lateral_index, 2U);
  EXPECT_EQ(selected840.tangent_index, 1U);
  EXPECT_NEAR(selected840.goal_d_m, -1.0, 1.0e-12);
  // Existing (0.50, 0.90) remains authoritative at Gen840. The robust region
  // must not replace any profile that already passes the unchanged evaluator.
  EXPECT_NEAR(selected840.dynamic_speed_limit_mps, 0.20116135163923135,
              1.0e-9);

  const auto generation_started = std::chrono::steady_clock::now();
  const auto generated841 = planner.generateCandidates(ego841, opponents841);
  const double generation_elapsed_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - generation_started)
          .count();
  RecordProperty("gen841_candidate_generation_ms", generation_elapsed_ms);
  const auto generated841_right = std::find_if(
      generated841.cbegin(), generated841.cend(), [](const auto &candidate) {
        return candidate.lateral_index == 2U && candidate.tangent_index == 1U;
      });
  ASSERT_NE(generated841_right, generated841.cend());
  ASSERT_TRUE(generated841_right->feasible)
      << generated841_right->rejection_reason
      << " dynamic_speed_limit_mps="
      << generated841_right->dynamic_speed_limit_mps
      << " dense_points=" << generated841_right->dense.size();
  EXPECT_NEAR(generated841_right->dynamic_speed_limit_mps,
              0.20171960386497198, 1.0e-9);
  EXPECT_LT(generation_elapsed_ms, config.planning_deadline_ms);

  const auto generation841 =
      planner.update(ego841, opponents841, true, 47.350);
  ASSERT_EQ(generation841.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation841.reason
      << " feasible=" << generation841.feasible_candidates
      << " trackability_rejects="
      << generation841.rejected_trackability_candidates;
  EXPECT_EQ(generation841.reason, "selected_right_candidate");
  EXPECT_TRUE(generation841.safe_lateral);
  EXPECT_FALSE(generation841.emergency_stop);
  EXPECT_TRUE(generation841.pass_continuation_active);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected841 = planner.selected().value();
  EXPECT_EQ(selected841.lateral_index, 2U);
  EXPECT_EQ(selected841.tangent_index, 1U);
  EXPECT_NEAR(selected841.goal_d_m, -1.0, 1.0e-12);
  EXPECT_NEAR(selected841.dynamic_speed_limit_mps, 0.20171960386497198,
              1.0e-9);
  EXPECT_LT(selected841.clearance_profile_attempt_index, 60U);
  EXPECT_GT(selected841.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  // The correction covers the measured continuous position interval rather
  // than special-casing the exact Gen841 coordinate.
  for (const double alpha : {0.0, 0.25, 0.50, 0.75, 1.0}) {
    auto interpolated_input = kCampaignV4Ego840;
    interpolated_input.stamp_sec = kCampaignV4Ego841.stamp_sec;
    interpolated_input.x_m +=
        alpha * (kCampaignV4Ego841.x_m - kCampaignV4Ego840.x_m);
    interpolated_input.y_m +=
        alpha * (kCampaignV4Ego841.y_m - kCampaignV4Ego840.y_m);
    const auto interpolated_ego =
        campaignTrackabilityEgo(interpolated_input, planning_frame);
    ASSERT_TRUE(interpolated_ego.valid);
    sl::LatticePlanner interval_planner(config, &planning_frame, &map,
                                        &output_frame);
    const auto interval_first =
        interval_planner.update(ego840, opponents840, true, 47.345);
    ASSERT_EQ(interval_first.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
        << "alpha=" << alpha << " reason=" << interval_first.reason;
    const auto interval_second = interval_planner.update(
        interpolated_ego, opponents840, true, 47.350);
    EXPECT_EQ(interval_second.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
        << "alpha=" << alpha << " reason=" << interval_second.reason
        << " feasible=" << interval_second.feasible_candidates;
  }

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded_execution = planner.boundedExactCartesianExecution(
      selected841, opponents841, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded_execution.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_EQ(bounded_diagnostic.failure, sl::OutputHorizonFailure::NONE);
  EXPECT_LE(bounded_execution->dense.size(), 256U);

  sl::OpponentState collision = opponents841.front();
  const auto &collision_point =
      selected841.dense.at(selected841.dense.size() / 2U);
  collision.id = "v4_robust_region_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected841, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);
}

TEST(LatticePlanner, CampaignV8Generation571To572TrackabilityBoundaryIsFailClosed) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego571 = campaignTrackabilityEgo(kCampaignV8Ego571, planning_frame);
  const auto ego572 = campaignTrackabilityEgo(kCampaignV8Ego572, planning_frame);
  const auto opponents571 = campaignTrackabilityOpponents(
      kCampaignV8Opponents571, planning_frame);
  const auto opponents572 = campaignTrackabilityOpponents(
      kCampaignV8Opponents572, planning_frame);
  ASSERT_TRUE(ego571.valid);
  ASSERT_TRUE(ego572.valid);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto generation571 = planner.update(ego571, opponents571, true, 34.170);
  ASSERT_EQ(generation571.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation571.reason;
  ASSERT_TRUE(planner.selected().has_value());
  EXPECT_GT(planner.selected()->dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);
  const double selected571_trackability_limit_mps =
      std::min(planner.selected()->entry_speed_limit_mps,
               planner.selected()->dynamic_speed_limit_mps);
  EXPECT_LE(generation571.speed_cap_mps,
            selected571_trackability_limit_mps + 1.0e-9);
  ASSERT_FALSE(generation571.speed_caps_mps.empty());
  EXPECT_TRUE(std::all_of(
      generation571.speed_caps_mps.cbegin(),
      generation571.speed_caps_mps.cend(),
      [selected571_trackability_limit_mps](const double speed_cap_mps) {
        return std::isfinite(speed_cap_mps) &&
               speed_cap_mps <= selected571_trackability_limit_mps + 1.0e-9;
      }));
  const auto selected571 = planner.selected().value();
  const auto nearest571 = std::min_element(
      selected571.dense.cbegin(), selected571.dense.cend(),
      [&ego572](const auto &lhs, const auto &rhs) {
        return std::hypot(lhs.x - ego572.x, lhs.y - ego572.y) <
               std::hypot(rhs.x - ego572.x, rhs.y - ego572.y);
      });
  ASSERT_NE(nearest571, selected571.dense.cend());
  RecordProperty("gen572_to_selected571_nearest_distance_m",
                 std::to_string(std::hypot(nearest571->x - ego572.x,
                                           nearest571->y - ego572.y)));
  RecordProperty("gen572_to_selected571_nearest_yaw_delta_rad",
                 std::to_string(std::remainder(
                     nearest571->yaw - ego572.yaw, 2.0 * M_PI)));
  RecordProperty("gen572_ego_curvature_radpm",
                 std::to_string(ego572.curvature));
  RecordProperty("selected571_nearest_curvature_radpm",
                 std::to_string(nearest571->kappa));
  RecordProperty("selected571_dynamic_speed_limit_mps",
                 std::to_string(selected571.dynamic_speed_limit_mps));

  const auto generation572 = planner.update(ego572, opponents572, true, 34.240);
  EXPECT_EQ(generation572.mode, sl::BehaviorMode::FOLLOW_BLOCKED)
      << generation572.reason
      << " feasible=" << generation572.feasible_candidates
      << " trackability=" << generation572.rejected_trackability_candidates;
  EXPECT_EQ(generation572.reason, "follow_blocked_no_feasible_pass_path");
  EXPECT_EQ(generation572.feasible_candidates, 0);
  EXPECT_GT(generation572.rejected_opponent_candidates, 0);
  EXPECT_FALSE(planner.selected().has_value());
}

TEST(LatticePlanner,
     CampaignV9Generation732UsesFinalBoundedProfileAfterExistingProfilesFail) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego731 = campaignTrackabilityEgo(kCampaignV9Ego731, planning_frame);
  const auto ego732 = campaignTrackabilityEgo(kCampaignV9Ego732, planning_frame);
  const auto opponents = campaignTrackabilityOpponents(
      kCampaignV9Opponents, planning_frame);
  ASSERT_TRUE(ego731.valid);
  ASSERT_TRUE(ego732.valid);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto generation731 = planner.update(ego731, opponents, true, 42.000);
  ASSERT_EQ(generation731.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation731.reason;
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected731 = planner.selected().value();
  EXPECT_EQ(selected731.lateral_index, 2U);
  EXPECT_EQ(selected731.tangent_index, 1U);
  EXPECT_EQ(generation731.feasible_candidates, 1);

  const auto generation732 = planner.update(ego732, opponents, true, 42.050);
  ASSERT_EQ(generation732.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation732.reason
      << " feasible=" << generation732.feasible_candidates;
  EXPECT_EQ(generation732.feasible_candidates, 1);
  EXPECT_GT(generation732.rejected_opponent_candidates, 0);
  EXPECT_TRUE(generation732.safe_lateral);
  EXPECT_FALSE(generation732.emergency_stop);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected732 = planner.selected().value();
  EXPECT_EQ(selected732.lateral_index, 2U);
  EXPECT_EQ(selected732.tangent_index, 1U);
  EXPECT_NEAR(selected732.dynamic_speed_limit_mps, 0.20266045824021056,
              1.0e-9);
  EXPECT_LT(selected732.clearance_profile_attempt_index, 60U);
  EXPECT_GT(selected732.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded_execution = planner.boundedExactCartesianExecution(
      selected732, opponents, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded_execution.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded_execution->dense.size(), 256U);

  sl::OpponentState collision = opponents.front();
  const auto &collision_point =
      selected732.dense.at(selected732.dense.size() / 2U);
  collision.id = "v9_gen732_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected732, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);
}

TEST(LatticePlanner,
     CampaignV11Generation559UsesFinalBoundedProfileRegionWithoutLimitChange) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego557 =
      campaignTrackabilityEgo(kCampaignV11Ego557, planning_frame);
  const auto ego558 =
      campaignTrackabilityEgo(kCampaignV11Ego558, planning_frame);
  const auto ego559 =
      campaignTrackabilityEgo(kCampaignV11Ego559, planning_frame);
  const auto opponents557_and_558 = campaignTrackabilityOpponents(
      kCampaignV11Opponents557And558, planning_frame);
  const auto opponents559 = campaignTrackabilityOpponents(
      kCampaignV11Opponents559, planning_frame);
  ASSERT_TRUE(ego557.valid);
  ASSERT_TRUE(ego558.valid);
  ASSERT_TRUE(ego559.valid);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto generation557 =
      planner.update(ego557, opponents557_and_558, true, 34.070);
  ASSERT_EQ(generation557.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation557.reason;

  const auto generation558 =
      planner.update(ego558, opponents557_and_558, true, 34.110);
  ASSERT_EQ(generation558.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation558.reason;
  ASSERT_EQ(generation558.feasible_candidates, 1);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected558 = planner.selected().value();
  EXPECT_EQ(selected558.lateral_index, 2U);
  EXPECT_EQ(selected558.tangent_index, 1U);
  EXPECT_NEAR(selected558.dynamic_speed_limit_mps, 0.20636242, 1.0e-7);

  const auto generation559 =
      planner.update(ego559, opponents559, true, 34.165);
  ASSERT_EQ(generation559.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation559.reason
      << " feasible=" << generation559.feasible_candidates
      << " trackability=" << generation559.rejected_trackability_candidates;
  EXPECT_EQ(generation559.feasible_candidates, 1);
  EXPECT_GT(generation559.rejected_opponent_candidates, 0);
  EXPECT_TRUE(generation559.safe_lateral);
  EXPECT_FALSE(generation559.emergency_stop);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected559 = planner.selected().value();
  EXPECT_EQ(selected559.lateral_index, 2U);
  EXPECT_EQ(selected559.tangent_index, 1U);
  EXPECT_NEAR(selected559.dynamic_speed_limit_mps, 0.20210403912791555,
              1.0e-9);
  EXPECT_LT(selected559.clearance_profile_attempt_index, 60U);
  EXPECT_GT(selected559.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded_execution = planner.boundedExactCartesianExecution(
      selected559, opponents559, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded_execution.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded_execution->dense.size(), 256U);

  sl::OpponentState collision = opponents559.front();
  const auto &collision_point =
      selected559.dense.at(selected559.dense.size() / 2U);
  collision.id = "v11_gen559_profile_region_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected559, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);
}

TEST(LatticePlanner,
     CampaignV12Generation702UsesFinalBoundedProfileRegionWithoutLimitChange) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego701 =
      campaignTrackabilityEgo(kCampaignV12Ego701, planning_frame);
  const auto ego702 =
      campaignTrackabilityEgo(kCampaignV12Ego702, planning_frame);
  const auto opponents701 = campaignTrackabilityOpponents(
      kCampaignV12Opponents701, planning_frame);
  const auto opponents702 = campaignTrackabilityOpponents(
      kCampaignV12Opponents702, planning_frame);
  ASSERT_TRUE(ego701.valid);
  ASSERT_TRUE(ego702.valid);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto generation701 = planner.update(ego701, opponents701, true, 41.295);
  ASSERT_EQ(generation701.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation701.reason;
  ASSERT_EQ(generation701.feasible_candidates, 1);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected701 = planner.selected().value();
  EXPECT_EQ(selected701.lateral_index, 2U);
  EXPECT_EQ(selected701.tangent_index, 1U);
  EXPECT_NEAR(selected701.dynamic_speed_limit_mps, 0.200942, 1.0e-6);

  const auto generation702 = planner.update(ego702, opponents702, true, 41.375);
  ASSERT_EQ(generation702.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation702.reason
      << " feasible=" << generation702.feasible_candidates
      << " trackability=" << generation702.rejected_trackability_candidates;
  EXPECT_EQ(generation702.feasible_candidates, 1);
  EXPECT_GT(generation702.rejected_opponent_candidates, 0);
  EXPECT_TRUE(generation702.safe_lateral);
  EXPECT_FALSE(generation702.emergency_stop);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected702 = planner.selected().value();
  EXPECT_EQ(selected702.lateral_index, 2U);
  EXPECT_EQ(selected702.tangent_index, 1U);
  EXPECT_NEAR(selected702.dynamic_speed_limit_mps, 0.2090329983948449,
              1.0e-9);
  EXPECT_LT(selected702.clearance_profile_attempt_index, 60U);
  EXPECT_GT(selected702.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded_execution = planner.boundedExactCartesianExecution(
      selected702, opponents702, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded_execution.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded_execution->dense.size(), 256U);

  sl::OpponentState collision = opponents702.front();
  const auto &collision_point =
      selected702.dense.at(selected702.dense.size() / 2U);
  collision.id = "v12_gen702_profile_region_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected702, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);
}

TEST(LatticePlanner,
     CampaignV16Gen701To702UsesFreshLowSpeedGeometricCurvatureContinuity) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  auto ego701 = campaignTrackabilityEgo(kCampaignV16Ego701, planning_frame);
  auto ego702 = campaignTrackabilityEgo(kCampaignV16Ego702, planning_frame);
  const auto opponents701 = campaignTrackabilityOpponents(
      kCampaignV16Opponents701, planning_frame);
  const auto opponents702 = campaignTrackabilityOpponents(
      kCampaignV16Opponents702, planning_frame);
  ASSERT_TRUE(ego701.valid);
  ASSERT_TRUE(ego702.valid);

  sl::LowSpeedCurvatureContinuity continuity;
  const double curvature_limit = std::min(
      sl::curvatureForSteering(config.planner_max_steer_rad,
                              config.wheel_base_m),
      config.reference_curvature_sanity_limit_radpm);
  EXPECT_FALSE(continuity
                   .update(ego701, ego701.stamp_sec, ego701.speed_mps,
                           ego701.yaw_rate_radps, curvature_limit,
                           config.ego_stale_sec)
                   .has_value());

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      planner, "d2", -1, 2U, 1U, -1.0, 9.2, 43.299999032, 0U, true,
      0.38, 0.81);
  const auto generation701 =
      planner.update(ego701, opponents701, true, ego701.stamp_sec);
  ASSERT_EQ(generation701.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation701.reason;
  ASSERT_EQ(generation701.feasible_candidates, 1);

  const auto curvature702 = continuity.update(
      ego702, ego702.stamp_sec, ego702.speed_mps, ego702.yaw_rate_radps,
      curvature_limit, config.ego_stale_sec);
  ASSERT_TRUE(curvature702.has_value());
  EXPECT_NEAR(curvature702.value(), -0.1344818943, 1.0e-8);
  ego702.curvature = curvature702.value();
  const auto generation702 =
      planner.update(ego702, opponents702, true, ego702.stamp_sec);
  ASSERT_EQ(generation702.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation702.reason
      << " feasible=" << generation702.feasible_candidates
      << " trackability=" << generation702.rejected_trackability_candidates;
  EXPECT_EQ(generation702.feasible_candidates, 1);
  EXPECT_GT(generation702.rejected_opponent_candidates, 0);
  EXPECT_TRUE(generation702.safe_lateral);
  EXPECT_FALSE(generation702.emergency_stop);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected702 = planner.selected().value();
  EXPECT_EQ(selected702.lateral_index, 2U);
  EXPECT_EQ(selected702.tangent_index, 1U);
  EXPECT_GT(selected702.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic diagnostic;
  const auto bounded = planner.boundedExactCartesianExecution(
      selected702, opponents702, 256U, &diagnostic);
  ASSERT_TRUE(bounded.has_value()) << sl::toString(diagnostic.failure);
  EXPECT_LE(bounded->dense.size(), 256U);
}

TEST(LatticePlanner,
     CampaignV13Generation598UsesFinalBoundedProfileRegionWithoutLimitChange) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego597 =
      campaignTrackabilityEgo(kCampaignV13PlannerInput597, planning_frame);
  const auto ego598 =
      campaignTrackabilityEgo(kCampaignV13PlannerInput598, planning_frame);
  const auto opponents597 = campaignTrackabilityOpponents(
      kCampaignV13Opponents597, planning_frame);
  const auto opponents598 = campaignTrackabilityOpponents(
      kCampaignV13Opponents598, planning_frame);
  ASSERT_TRUE(ego597.valid);
  ASSERT_TRUE(ego598.valid);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      planner, "d2", -1, 2U, 1U, -1.0, 8.733, 36.650, 596U);
  const auto generation597 = planner.update(ego597, opponents597, true, 36.805);
  ASSERT_EQ(generation597.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation597.reason;
  ASSERT_GT(generation597.feasible_candidates, 0);
  ASSERT_TRUE(planner.selected().has_value());

  const auto generation598 = planner.update(ego598, opponents598, true, 36.845);
  ASSERT_EQ(generation598.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation598.reason;
  EXPECT_GT(generation598.feasible_candidates, 0);
  EXPECT_GT(generation598.rejected_opponent_candidates, 0);
  EXPECT_TRUE(generation598.safe_lateral);
  EXPECT_FALSE(generation598.emergency_stop);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected598 = planner.selected().value();
  EXPECT_EQ(selected598.lateral_index, 2U);
  EXPECT_EQ(selected598.tangent_index, 1U);
  EXPECT_GT(selected598.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded_execution = planner.boundedExactCartesianExecution(
      selected598, opponents598, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded_execution.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded_execution->dense.size(), 256U);

  sl::OpponentState collision = opponents598.front();
  const auto &collision_point =
      selected598.dense.at(selected598.dense.size() / 2U);
  collision.id = "v13_gen598_profile_region_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected598, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);
}

TEST(LatticePlanner,
     CampaignV10Generation591UsesBoundedProfileRegionAtSelectedSteeringRate) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego590 =
      campaignTrackabilityEgo(kCampaignV10Ego590, planning_frame);
  const auto ego591 =
      campaignTrackabilityEgo(kCampaignV10Ego591, planning_frame);
  const auto opponents590 = campaignTrackabilityOpponents(
      kCampaignV10Opponents590, planning_frame);
  const auto opponents591 = campaignTrackabilityOpponents(
      kCampaignV10Opponents591, planning_frame);
  ASSERT_TRUE(ego590.valid);
  ASSERT_TRUE(ego591.valid);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  const auto generation590 = planner.update(ego590, opponents590, true, 37.735);
  ASSERT_EQ(generation590.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation590.reason;
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected590 = planner.selected().value();
  EXPECT_EQ(selected590.lateral_index, 2U);
  EXPECT_EQ(selected590.tangent_index, 1U);

  const auto generated591 = planner.generateCandidates(ego591, opponents591);
  const auto generated591_feasible = std::count_if(
      generated591.cbegin(), generated591.cend(),
      [](const auto &candidate) { return candidate.feasible; });
  EXPECT_EQ(generated591_feasible, 1U);

  const auto generation591 = planner.update(ego591, opponents591, true, 37.850);
  ASSERT_EQ(generation591.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation591.reason
      << " feasible=" << generation591.feasible_candidates
      << " trackability=" << generation591.rejected_trackability_candidates;
  EXPECT_EQ(generation591.feasible_candidates, 1);
  EXPECT_GT(generation591.rejected_opponent_candidates, 0);
  EXPECT_TRUE(generation591.safe_lateral);
  EXPECT_FALSE(generation591.emergency_stop);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected591 = planner.selected().value();
  EXPECT_EQ(selected591.lateral_index, 2U);
  EXPECT_EQ(selected591.tangent_index, 1U);
  EXPECT_GT(selected591.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded_execution = planner.boundedExactCartesianExecution(
      selected591, opponents591, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded_execution.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded_execution->dense.size(), 256U);

  sl::OpponentState collision = opponents591.front();
  const auto &collision_point =
      selected591.dense.at(selected591.dense.size() / 2U);
  collision.id = "v10_gen591_profile_region_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected591, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);
}

TEST(LatticePlanner,
     CampaignV19Generation445UsesBoundedTangentContinuityAfterExistingSearch) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  ASSERT_DOUBLE_EQ(config.planning_deadline_ms, 80.0);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego444 =
      campaignTrackabilityEgo(kCampaignV19Ego444, planning_frame);
  const auto ego445 =
      campaignTrackabilityEgo(kCampaignV19Ego445, planning_frame);
  const auto opponents444 = campaignTrackabilityOpponents(
      kCampaignV19Opponents444, planning_frame);
  const auto opponents445 = campaignTrackabilityOpponents(
      kCampaignV19Opponents445, planning_frame);
  ASSERT_TRUE(ego444.valid);
  ASSERT_TRUE(ego445.valid);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      planner, "d2", -1, 2U, 1U, -1.0, 10.0, 27.949999375, 443U, true,
      0.45, 0.90, 1.0);
  const auto generation444 =
      planner.update(ego444, opponents444, true, ego444.stamp_sec + 0.015);
  ASSERT_EQ(generation444.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation444.reason;
  ASSERT_EQ(generation444.feasible_candidates, 1);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected444 = planner.selected().value();
  ASSERT_TRUE(selected444.clearance_profile_applied);
  EXPECT_NEAR(selected444.clearance_profile_join_fraction, 0.46, 1.0e-12);
  EXPECT_NEAR(selected444.clearance_profile_yaw_magnitude, 0.92, 1.0e-12);
  EXPECT_NEAR(selected444.clearance_profile_tangent_scale, 1.0, 1.0e-12);

  const auto started = std::chrono::steady_clock::now();
  const auto generation445 =
      planner.update(ego445, opponents445, true, ego445.stamp_sec + 0.015);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
  RecordProperty("gen445_planner_update_ms", elapsed_ms);
  ASSERT_EQ(generation445.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation445.reason
      << " wall=" << generation445.rejected_wall_candidates
      << " opponent=" << generation445.rejected_opponent_candidates
      << " trackability=" << generation445.rejected_trackability_candidates;
  EXPECT_EQ(generation445.feasible_candidates, 1);
  EXPECT_EQ(generation445.rejected_wall_candidates, 6);
  EXPECT_EQ(generation445.rejected_opponent_candidates, 8);
  EXPECT_LT(elapsed_ms, config.planning_deadline_ms);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected445 = planner.selected().value();
  EXPECT_EQ(selected445.lateral_index, 2U);
  EXPECT_EQ(selected445.tangent_index, 1U);
  EXPECT_TRUE(selected445.clearance_profile_applied);
  EXPECT_NEAR(selected445.clearance_profile_join_fraction, 0.46, 1.0e-12);
  EXPECT_NEAR(selected445.clearance_profile_yaw_magnitude, 0.92, 1.0e-12);
  EXPECT_NEAR(selected445.clearance_profile_tangent_scale, 1.01, 1.0e-12);
  EXPECT_NEAR(selected445.tangent_scale, 1.01, 1.0e-12);
  EXPECT_GT(selected445.dynamic_speed_limit_mps,
            config.safe_stop_speed_mps);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded = planner.boundedExactCartesianExecution(
      selected445, opponents445, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded->dense.size(), 256U);

  sl::OpponentState collision = opponents445.front();
  const auto &collision_point =
      selected445.dense.at(selected445.dense.size() / 2U);
  collision.id = "v19_tangent_continuity_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet =
      planning_frame.project(collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected445, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);

  sl::LatticePlanner invalid_hint_planner(
      config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      invalid_hint_planner, "d2", -1, 2U, 1U, -1.0, 10.0,
      27.999999374, 444U, true, 0.46, 0.92, 0.0);
  const auto invalid_hint_generation = invalid_hint_planner.update(
      ego445, opponents445, true, ego445.stamp_sec + 0.015);
  EXPECT_EQ(invalid_hint_generation.mode, sl::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(invalid_hint_generation.feasible_candidates, 0);
  EXPECT_FALSE(invalid_hint_planner.selected().has_value());
}

TEST(LatticePlanner,
     CampaignV21DeadlineClearsAuthorityButKeepsOneShotBoundedSearchHint) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.planning_deadline_ms, 80.0);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego499 =
      campaignTrackabilityEgo(kCampaignV21Ego499, planning_frame);
  const auto ego500 =
      campaignTrackabilityEgo(kCampaignV21Ego500, planning_frame);
  const auto ego501 =
      campaignTrackabilityEgo(kCampaignV21Ego501, planning_frame);
  const auto opponents499 = campaignTrackabilityOpponents(
      kCampaignV21Opponents499, planning_frame);
  const auto opponents500And501 = campaignTrackabilityOpponents(
      kCampaignV21Opponents500And501, planning_frame);
  ASSERT_TRUE(ego499.valid);
  ASSERT_TRUE(ego500.valid);
  ASSERT_TRUE(ego501.valid);

  sl::LatticePlanner committed(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      committed, "d2", -1, 2U, 1U, -1.0, 9.575, 30.349999321,
      498U, true, 0.34, 0.82, 1.0);
  const auto generation499 =
      committed.update(ego499, opponents499, true, 30.569999316);
  ASSERT_EQ(generation499.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation499.reason;
  ASSERT_EQ(generation499.feasible_candidates, 1);
  ASSERT_TRUE(committed.selected().has_value());
  EXPECT_EQ(generation499.rejected_wall_candidates, 4);
  EXPECT_EQ(generation499.rejected_opponent_candidates, 10);
  const auto selected499 = *committed.selected();
  ASSERT_TRUE(selected499.clearance_profile_applied);
  EXPECT_NEAR(selected499.clearance_profile_join_fraction, 0.34, 1.0e-12);
  EXPECT_NEAR(selected499.clearance_profile_yaw_magnitude, 0.82, 1.0e-12);
  EXPECT_NEAR(selected499.clearance_profile_tangent_scale, 1.0, 1.0e-12);

  auto uncommitted_trial = committed;
  const auto trial_started = std::chrono::steady_clock::now();
  const auto generation500 = uncommitted_trial.update(
      ego500, opponents500And501, true, 30.659999314);
  const double generation500_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - trial_started)
          .count();
  RecordProperty("gen500_core_update_ms", generation500_ms);
  ASSERT_EQ(generation500.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation500.reason;
  EXPECT_EQ(generation500.feasible_candidates, 1);
  EXPECT_LT(generation500_ms, config.planning_deadline_ms);

  auto after_deadline = committed;
  after_deadline.clearPassContinuationAuthorityPreservingSearchHint();
  EXPECT_FALSE(
      sl::StateLatticeTestAccess::hasPassContinuationAuthority(after_deadline));
  ASSERT_TRUE(
      sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(after_deadline));
  const auto generation501_started = std::chrono::steady_clock::now();
  const auto generation501 = after_deadline.update(
      ego501, opponents500And501, true, 30.709999313);
  const double generation501_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - generation501_started)
          .count();
  RecordProperty("gen501_deadline_recovery_core_update_ms", generation501_ms);
  ASSERT_EQ(generation501.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation501.reason;
  EXPECT_EQ(generation501.generated_candidates, 1);
  EXPECT_EQ(generation501.feasible_candidates, 1);
  EXPECT_EQ(generation501.rejected_wall_candidates, 0);
  EXPECT_EQ(generation501.rejected_opponent_candidates, 0);
  EXPECT_LT(generation501_ms, config.planning_deadline_ms);
  EXPECT_FALSE(
      sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(after_deadline));
  ASSERT_TRUE(after_deadline.selected().has_value());
  EXPECT_TRUE(after_deadline.lastPlanningCycleMetrics()
                  .accepted_path_continuation_accepted);
  EXPECT_TRUE(
      sl::StateLatticeTestAccess::hasPassContinuationAuthority(after_deadline));
  EXPECT_TRUE(sl::StateLatticeTestAccess::hasAcceptedPathContinuationHint(
      after_deadline))
      << "the consumed deadline hint must not erase the newly admitted hint";
  const auto selected501 = *after_deadline.selected();
  EXPECT_TRUE(selected501.clearance_profile_applied);
  EXPECT_NEAR(selected501.clearance_profile_join_fraction, 0.34, 1.0e-12);
  EXPECT_NEAR(selected501.clearance_profile_yaw_magnitude, 0.82, 1.0e-12);
  EXPECT_NEAR(selected501.clearance_profile_tangent_scale, 1.0, 1.0e-12);
  EXPECT_EQ(selected501.clearance_profile_attempt_index, 0U)
      << "the exact accepted center must be the first bounded attempt";
  EXPECT_EQ(selected501.clearance_profile_attempts_evaluated, 1U);
  EXPECT_LE(selected501.clearance_profile_unique_attempt_limit, 167U);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded = after_deadline.boundedExactCartesianExecution(
      selected501, opponents500And501, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded->dense.size(), 256U);

  sl::OpponentState collision = opponents500And501.front();
  const auto &collision_point =
      selected501.dense.at(selected501.dense.size() / 2U);
  collision.id = "v21_deadline_hint_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet = planning_frame.project(
      collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(after_deadline
                   .boundedExactCartesianExecution(
                       selected501, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);

  auto backward_stamp = committed;
  backward_stamp.clearPassContinuationAuthorityPreservingSearchHint();
  auto backward_opponents = opponents500And501;
  for (auto &opponent : backward_opponents) {
    opponent.stamp_sec = 30.299999322;
  }
  const auto backward_output = backward_stamp.update(
      ego501, backward_opponents, true, 30.709999313);
  EXPECT_EQ(backward_output.mode, sl::BehaviorMode::FOLLOW_BLOCKED)
      << backward_output.reason;
  EXPECT_EQ(backward_output.feasible_candidates, 0);
  EXPECT_FALSE(backward_stamp.selected().has_value());
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(
      backward_stamp));

  auto wrong_identity = committed;
  wrong_identity.clearPassContinuationAuthorityPreservingSearchHint();
  auto wrong_identity_opponents = opponents500And501;
  ASSERT_FALSE(wrong_identity_opponents.empty());
  wrong_identity_opponents.front().id = "different_current_blocker";
  (void)wrong_identity.update(
      ego501, wrong_identity_opponents, true, 30.709999313);
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasPassContinuationAuthority(
      wrong_identity));
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(
      wrong_identity));

  auto wrong_side = committed;
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      wrong_side, "d2", 1, 2U, 1U, -1.0, 9.575, 30.399999320,
      499U, true, 0.34, 0.82, 1.0);
  wrong_side.clearPassContinuationAuthorityPreservingSearchHint();
  (void)wrong_side.update(
      ego501, opponents500And501, true, 30.709999313);
  EXPECT_FALSE(
      sl::StateLatticeTestAccess::hasPassContinuationAuthority(wrong_side));
  EXPECT_FALSE(
      sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(wrong_side));

  auto repeated_deadline = committed;
  repeated_deadline.clearPassContinuationAuthorityPreservingSearchHint();
  ASSERT_TRUE(sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(
      repeated_deadline));
  repeated_deadline.clearPassContinuationAuthorityPreservingSearchHint();
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(
      repeated_deadline));

  sl::LatticePlanner nonfinite(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      nonfinite, "d2", -1, 2U, 1U, -1.0, 9.575, 30.399999320,
      499U, true, std::numeric_limits<double>::quiet_NaN(), 0.82, 1.0);
  nonfinite.clearPassContinuationAuthorityPreservingSearchHint();
  EXPECT_FALSE(
      sl::StateLatticeTestAccess::hasPassContinuationAuthority(nonfinite));
  EXPECT_FALSE(
      sl::StateLatticeTestAccess::hasDeadlineProfileSearchHint(nonfinite));
}

TEST(LatticePlanner,
     CampaignV22AcceptedProfileTangentRadiusPreservesGen456AndClosesGen457) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.planning_deadline_ms, 80.0);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const auto ego456 =
      campaignTrackabilityEgo(kCampaignV22Ego456, planning_frame);
  const auto ego457 =
      campaignTrackabilityEgo(kCampaignV22Ego457, planning_frame);
  const auto opponents456 = campaignTrackabilityOpponents(
      kCampaignV22Opponents456, planning_frame);
  const auto opponents457 = campaignTrackabilityOpponents(
      kCampaignV22Opponents457, planning_frame);
  ASSERT_TRUE(ego456.valid);
  ASSERT_TRUE(ego457.valid);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      planner, "d2", -1, 2U, 1U, -1.0, 9.224, 28.099999371,
      455U, true, 0.28, 0.68, 1.0);

  const auto generation456_started = std::chrono::steady_clock::now();
  const auto generation456 =
      planner.update(ego456, opponents456, true, 28.255);
  const double generation456_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - generation456_started)
          .count();
  RecordProperty("gen456_core_update_ms", generation456_ms);
  ASSERT_EQ(generation456.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation456.reason;
  ASSERT_EQ(generation456.feasible_candidates, 1);
  EXPECT_EQ(generation456.rejected_wall_candidates, 4);
  EXPECT_EQ(generation456.rejected_opponent_candidates, 10);
  EXPECT_LT(generation456_ms, config.planning_deadline_ms);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected456 = *planner.selected();
  ASSERT_TRUE(selected456.clearance_profile_applied);
  EXPECT_NEAR(selected456.clearance_profile_join_fraction, 0.27, 1.0e-12);
  EXPECT_NEAR(selected456.clearance_profile_yaw_magnitude, 0.71, 1.0e-12);
  EXPECT_NEAR(selected456.clearance_profile_tangent_scale, 1.0, 1.0e-12);
  // This selected outer-ring profile was index 35 when all 49 continuity
  // profiles incorrectly preceded the global list. Requiring post-global
  // placement distinguishes the authorized 9-profile prefix.
  EXPECT_GE(selected456.clearance_profile_attempt_index, 69U);
  EXPECT_LT(selected456.clearance_profile_attempt_index, 167U);

  const auto generation457_started = std::chrono::steady_clock::now();
  const auto generation457 =
      planner.update(ego457, opponents457, true, 28.355);
  const double generation457_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - generation457_started)
          .count();
  RecordProperty("gen457_162_attempt_core_update_ms", generation457_ms);
  ASSERT_EQ(generation457.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation457.reason;
  ASSERT_EQ(generation457.feasible_candidates, 1);
  EXPECT_EQ(generation457.rejected_wall_candidates, 4);
  EXPECT_EQ(generation457.rejected_opponent_candidates, 10);
  EXPECT_LT(generation457_ms, config.planning_deadline_ms);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected457 = *planner.selected();
  ASSERT_TRUE(selected457.clearance_profile_applied);
  EXPECT_NEAR(selected457.clearance_profile_join_fraction, 0.26, 1.0e-12);
  EXPECT_NEAR(selected457.clearance_profile_yaw_magnitude, 0.71, 1.0e-12);
  EXPECT_NEAR(selected457.clearance_profile_tangent_scale, 1.02, 1.0e-12);
  // Only the exact accepted-profile inner 3x3 may precede the established
  // 60-profile list. The tangent-radius extension must remain post-global.
  EXPECT_GE(selected457.clearance_profile_attempt_index, 69U);
  EXPECT_LT(selected457.clearance_profile_attempt_index, 167U);
  EXPECT_LE(selected457.clearance_profile_attempts_evaluated, 167U);
  EXPECT_LE(selected457.clearance_profile_unique_attempt_limit, 167U);

  sl::OutputHorizonDiagnostic bounded_diagnostic;
  const auto bounded = planner.boundedExactCartesianExecution(
      selected457, opponents457, 256U, &bounded_diagnostic);
  ASSERT_TRUE(bounded.has_value())
      << sl::toString(bounded_diagnostic.failure);
  EXPECT_LE(bounded->dense.size(), 256U);

  sl::OpponentState collision = opponents457.front();
  const auto &collision_point =
      selected457.dense.at(selected457.dense.size() / 2U);
  collision.id = "v22_tangent_radius_hard_negative";
  collision.x = collision_point.x;
  collision.y = collision_point.y;
  collision.yaw = collision_point.yaw;
  collision.vx_mps = 0.0;
  collision.vy_mps = 0.0;
  collision.speed_mps = 0.0;
  collision.frenet = planning_frame.project(
      collision.x, collision.y, collision.yaw);
  collision.valid = collision.frenet.valid;
  ASSERT_TRUE(collision.valid);
  sl::OutputHorizonDiagnostic hard_negative_diagnostic;
  EXPECT_FALSE(planner
                   .boundedExactCartesianExecution(
                       selected457, {collision}, 256U,
                       &hard_negative_diagnostic)
                   .has_value());
  EXPECT_TRUE(hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION ||
              hard_negative_diagnostic.failure ==
                  sl::OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION);

  sl::LatticePlanner wrong_identity(
      config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      wrong_identity, "different_blocker", -1, 2U, 1U, -1.0, 9.224,
      28.099999371, 455U, true, 0.27, 0.71, 1.0);
  (void)wrong_identity.update(ego457, opponents457, true, 28.355);
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasPassContinuationAuthority(
      wrong_identity));
  EXPECT_FALSE(wrong_identity.selected().has_value());

  sl::LatticePlanner wrong_stamp(
      config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      wrong_stamp, "d2", -1, 2U, 1U, -1.0, 9.224, 28.299999367,
      455U, true, 0.27, 0.71, 1.0);
  (void)wrong_stamp.update(ego457, opponents457, true, 28.355);
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasPassContinuationAuthority(
      wrong_stamp));
  EXPECT_FALSE(wrong_stamp.selected().has_value());

  sl::LatticePlanner wrong_side(
      config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      wrong_side, "d2", 1, 2U, 1U, -1.0, 9.224, 28.099999371,
      455U, true, 0.27, 0.71, 1.0);
  (void)wrong_side.update(ego457, opponents457, true, 28.355);
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasPassContinuationAuthority(
      wrong_side));
  EXPECT_FALSE(wrong_side.selected().has_value());

  sl::LatticePlanner wrong_branch(
      config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      wrong_branch, "d2", -1, 99U, 99U, -1.0, 9.224, 28.099999371,
      455U, true, 0.27, 0.71, 1.0);
  (void)wrong_branch.update(ego457, opponents457, true, 28.355);
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasPassContinuationAuthority(
      wrong_branch));
  EXPECT_FALSE(wrong_branch.selected().has_value());
}

TEST(LatticePlanner,
     CampaignV23AcceptedProfileFastPathPreservesGen551AndFailsClosedGen552) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.planning_deadline_ms, 80.0);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  ASSERT_TRUE(sl::validateConfig(config).empty());
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  auto ego551 = campaignTrackabilityEgo(kCampaignV23Ego551, planning_frame);
  auto ego552 = campaignTrackabilityEgo(kCampaignV23Ego552, planning_frame);
  ego551.curvature = -0.16907898477750993;
  ego552.curvature = -0.0459643581162486;
  ego551.valid = ego551.frenet.valid && std::isfinite(ego551.curvature);
  ego552.valid = ego552.frenet.valid && std::isfinite(ego552.curvature);
  const auto opponents551 = campaignTrackabilityOpponents(
      kCampaignV23Opponents551, planning_frame);
  const auto opponents552 = campaignTrackabilityOpponents(
      kCampaignV23Opponents552, planning_frame);
  ASSERT_TRUE(ego551.valid);
  ASSERT_TRUE(ego552.valid);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      planner, "d2", -1, 2U, 1U, -1.0, 9.548, 33.099999260,
      550U, true, 0.31, 0.80, 1.01);

  const auto generation551_started = std::chrono::steady_clock::now();
  const auto generation551 =
      planner.update(ego551, opponents551, true, 33.264999259);
  const double generation551_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - generation551_started)
          .count();
  RecordProperty("gen551_fast_path_core_update_ms", generation551_ms);
  ASSERT_EQ(generation551.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation551.reason;
  ASSERT_EQ(generation551.feasible_candidates, 1);
  ASSERT_TRUE(planner.selected().has_value());
  const auto selected551 = *planner.selected();
  ASSERT_TRUE(selected551.clearance_profile_applied);
  EXPECT_NEAR(selected551.clearance_profile_join_fraction, 0.31, 1.0e-12);
  EXPECT_NEAR(selected551.clearance_profile_yaw_magnitude, 0.80, 1.0e-12);
  EXPECT_NEAR(selected551.clearance_profile_tangent_scale, 1.01, 1.0e-12);
  EXPECT_EQ(selected551.clearance_profile_attempt_index, 0U);
  EXPECT_EQ(selected551.clearance_profile_attempts_evaluated, 1U);
  EXPECT_LE(selected551.clearance_profile_unique_attempt_limit, 167U);
  EXPECT_LT(generation551_ms, config.planning_deadline_ms);

  const auto generation552_started = std::chrono::steady_clock::now();
  const auto generation552 =
      planner.update(ego552, opponents552, true, 33.364999256);
  const double generation552_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - generation552_started)
          .count();
  RecordProperty("gen552_complete_fallback_core_update_ms", generation552_ms);
  EXPECT_EQ(generation552.mode, sl::BehaviorMode::FOLLOW_BLOCKED)
      << generation552.reason;
  EXPECT_EQ(generation552.feasible_candidates, 0);
  EXPECT_FALSE(planner.selected().has_value());
  EXPECT_LT(generation552_ms, config.planning_deadline_ms);
  ASSERT_EQ(planner.candidates().size(), 15U);
  const auto branch = std::find_if(
      planner.candidates().begin(), planner.candidates().end(),
      [](const sl::CandidateTrajectory &candidate) {
        return candidate.lateral_index == 2U &&
               candidate.tangent_index == 1U;
      });
  ASSERT_NE(branch, planner.candidates().end());
  EXPECT_GT(branch->clearance_profile_attempts_evaluated, 60U);
  EXPECT_EQ(branch->clearance_profile_attempts_evaluated,
            branch->clearance_profile_unique_attempt_limit);
  EXPECT_LE(branch->clearance_profile_unique_attempt_limit, 167U);
}

TEST(LatticePlanner,
     CampaignV24AcceptedPathContinuationClosesGen526To527RecedingHorizon) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  const auto config = campaignTrackabilityConfig(planning_frame);
  ASSERT_DOUBLE_EQ(config.planning_deadline_ms, 80.0);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const CampaignTrackabilityEgoInput input526{
      32.039999283, 89631.1390034061, 43128.799225303585,
      0.8039350142471839, 0.5947171536683469, 0.1417830636140276,
      -0.0233187614887893};
  const CampaignTrackabilityEgoInput input527{
      32.139999281, 89631.13461786584, 43128.81265528785,
      0.8033135856896348, 0.5955562803350005, 0.14118527345713322,
      -0.022849966444295936};
  const std::array<CampaignTrackabilityOpponentInput, 3U> opponents526_raw{{
      {"d2", 31.949999285, 89628.9140625, 43131.0, 31.999999284,
       89628.9140625, 43131.0},
      {"d3", 31.949999285, 89624.7265625, 43137.74609375, 31.999999284,
       89624.7265625, 43137.74609375},
      {"d4", 31.949999285, 89620.234375, 43144.671875, 31.999999284,
       89620.234375, 43144.671875},
  }};
  const std::array<CampaignTrackabilityOpponentInput, 3U> opponents527_raw{{
      {"d2", 32.099999282, 89628.9140625, 43131.0, 32.149999281,
       89628.9140625, 43131.0},
      {"d3", 32.099999282, 89624.7265625, 43137.74609375, 32.149999281,
       89624.7265625, 43137.74609375},
      {"d4", 32.099999282, 89620.234375, 43144.671875, 32.149999281,
       89620.234375, 43144.671875},
  }};
  auto ego526 = campaignTrackabilityEgo(input526, planning_frame);
  auto ego527 = campaignTrackabilityEgo(input527, planning_frame);
  ego526.curvature = -0.170873;
  ego527.curvature = -0.170255;
  const auto opponents526 =
      campaignTrackabilityOpponents(opponents526_raw, planning_frame);
  const auto opponents527 =
      campaignTrackabilityOpponents(opponents527_raw, planning_frame);

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      planner, "d2", -1, 2U, 1U, -1.0, 9.419, 31.999999284, 525U,
      true, 0.30, 0.78, 1.02);
  const auto generation526 =
      planner.update(ego526, opponents526, true, 32.154999281);
  ASSERT_EQ(generation526.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation526.reason;
  ASSERT_TRUE(planner.selected().has_value());
  ASSERT_TRUE(sl::StateLatticeTestAccess::hasAcceptedPathContinuationHint(
      planner));
  EXPECT_FALSE(planner.lastPlanningCycleMetrics()
                   .accepted_path_continuation_requested);
  EXPECT_FALSE(planner.lastPlanningCycleMetrics()
                   .accepted_path_continuation_evaluated);

  const auto generation527_started = std::chrono::steady_clock::now();
  sl::LatticePlanner generation527_trial(planner);
  generation527_trial.clearPassContinuationAuthorityPreservingSearchHint();
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasPassContinuationAuthority(
      generation527_trial));
  ASSERT_TRUE(sl::StateLatticeTestAccess::hasAcceptedPathContinuationHint(
      generation527_trial));
  const auto generation527 =
      generation527_trial.update(ego527, opponents527, true, 32.204999280);
  const double generation527_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - generation527_started)
          .count();
  RecordProperty("gen527_receding_horizon_core_update_ms", generation527_ms);
  ASSERT_EQ(generation527.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation527.reason;
  EXPECT_EQ(generation527.generated_candidates, 1);
  EXPECT_EQ(generation527.feasible_candidates, 1);
  EXPECT_TRUE(sl::StateLatticeTestAccess::hasPassContinuationAuthority(
      generation527_trial));
  ASSERT_TRUE(generation527_trial.selected().has_value());
  const auto &selected527 = *generation527_trial.selected();
  ASSERT_FALSE(selected527.dense.empty());
  EXPECT_NEAR(selected527.dense.front().x, ego527.x, 1.0e-9);
  EXPECT_NEAR(selected527.dense.front().y, ego527.y, 1.0e-9);
  EXPECT_NEAR(selected527.dense.front().yaw, ego527.yaw, 1.0e-9);
  EXPECT_TRUE(
      generation527_trial
          .boundedExactCartesianExecution(selected527, opponents527, 256U)
          .has_value());
  const auto &continuation_diagnostic =
      generation527_trial.lastPlanningCycleMetrics();
  EXPECT_TRUE(continuation_diagnostic.accepted_path_continuation_requested);
  EXPECT_TRUE(
      continuation_diagnostic.accepted_path_continuation_target_matches);
  EXPECT_TRUE(continuation_diagnostic.accepted_path_continuation_evaluated);
  EXPECT_TRUE(
      continuation_diagnostic.accepted_path_continuation_exact_binding_valid);
  EXPECT_TRUE(continuation_diagnostic.accepted_path_continuation_anchor_valid);
  EXPECT_GT(
      continuation_diagnostic.accepted_path_continuation_connector_attempts,
      0U);
  EXPECT_TRUE(continuation_diagnostic.accepted_path_continuation_accepted);
  EXPECT_EQ(continuation_diagnostic.accepted_path_continuation_stage,
            "accepted");
  EXPECT_LT(generation527_ms, config.planning_deadline_ms);

  sl::LatticePlanner stale_target(config, &planning_frame, &map,
                                  &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      stale_target, "d2", -1, 2U, 1U, -1.0, 9.419, 31.999999284, 525U,
      true, 0.30, 0.78, 1.02);
  ASSERT_EQ(stale_target.update(ego526, opponents526, true, 32.154999281).mode,
            sl::BehaviorMode::OVERTAKE_RIGHT);
  stale_target.clearPassContinuationAuthorityPreservingSearchHint();
  EXPECT_FALSE(
      sl::StateLatticeTestAccess::hasPassContinuationAuthority(stale_target));
  auto stale_opponents = opponents527;
  stale_opponents.front().stamp_sec = 31.899999284;
  const auto stale_output =
      stale_target.update(ego527, stale_opponents, true, 32.204999280);
  EXPECT_NE(stale_output.mode, sl::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_FALSE(stale_target.selected().has_value());
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasAcceptedPathContinuationHint(
      stale_target));
  EXPECT_TRUE(stale_target.lastPlanningCycleMetrics()
                  .accepted_path_continuation_requested);
  EXPECT_FALSE(stale_target.lastPlanningCycleMetrics()
                   .accepted_path_continuation_target_matches);
  EXPECT_FALSE(stale_target.lastPlanningCycleMetrics()
                   .accepted_path_continuation_evaluated);
  EXPECT_EQ(stale_target.lastPlanningCycleMetrics()
                .accepted_path_continuation_stage,
            "entry_target_binding_rejected");

  sl::LatticePlanner duplicate_identity(planner);
  duplicate_identity.clearPassContinuationAuthorityPreservingSearchHint();
  auto duplicate_opponents = opponents527;
  auto invalid_duplicate = duplicate_opponents.front();
  invalid_duplicate.valid = false;
  duplicate_opponents.push_back(invalid_duplicate);
  const auto duplicate_output = duplicate_identity.update(
      ego527, duplicate_opponents, true, 32.204999280);
  EXPECT_EQ(duplicate_output.generated_candidates, 15);
  EXPECT_FALSE(duplicate_identity.lastPlanningCycleMetrics()
                   .accepted_path_continuation_target_matches);
  EXPECT_FALSE(duplicate_identity.lastPlanningCycleMetrics()
                   .accepted_path_continuation_evaluated);
  EXPECT_EQ(duplicate_identity.lastPlanningCycleMetrics()
                .accepted_path_continuation_stage,
            "entry_target_binding_rejected");

  sl::LatticePlanner ambiguous_join(config, &planning_frame, &map,
                                    &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      ambiguous_join, "d2", -1, 2U, 1U, -1.0, 9.419, 31.999999284, 525U,
      true, 0.30, 0.78, 1.02);
  ASSERT_EQ(
      ambiguous_join.update(ego526, opponents526, true, 32.154999281).mode,
      sl::BehaviorMode::OVERTAKE_RIGHT);
  ambiguous_join.clearPassContinuationAuthorityPreservingSearchHint();
  sl::StateLatticeTestAccess::makeAcceptedPathContinuationJoinAmbiguous(
      ambiguous_join);
  const auto ambiguous_output =
      ambiguous_join.update(ego527, opponents527, true, 32.204999280);
  EXPECT_NE(ambiguous_output.mode, sl::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_FALSE(ambiguous_join.selected().has_value());
  EXPECT_EQ(ambiguous_output.generated_candidates, 15);
  EXPECT_EQ(ambiguous_join.lastPlanningCycleMetrics()
                .accepted_path_continuation_stage,
            "forward_anchor_ambiguous");

  sl::LatticePlanner collision_guard(config, &planning_frame, &map,
                                     &output_frame);
  sl::StateLatticeTestAccess::seedPassContinuationLatch(
      collision_guard, "d2", -1, 2U, 1U, -1.0, 9.419, 31.999999284, 525U,
      true, 0.30, 0.78, 1.02);
  ASSERT_EQ(
      collision_guard.update(ego526, opponents526, true, 32.154999281).mode,
      sl::BehaviorMode::OVERTAKE_RIGHT);
  ASSERT_TRUE(collision_guard.selected().has_value());
  const auto &prior_dense = collision_guard.selected()->dense;
  ASSERT_GT(prior_dense.size(), 4U);
  const auto &blocked_point = prior_dense[prior_dense.size() * 2U / 3U];
  auto collision_opponents = opponents527;
  sl::OpponentState path_blocker;
  path_blocker.id = "path_blocker";
  path_blocker.stamp_sec = opponents527.front().stamp_sec;
  path_blocker.x = blocked_point.x;
  path_blocker.y = blocked_point.y;
  path_blocker.yaw = blocked_point.yaw;
  path_blocker.speed_mps = 0.0;
  path_blocker.uncertainty_x_m = 0.15;
  path_blocker.uncertainty_y_m = 0.15;
  path_blocker.frenet = planning_frame.project(
      path_blocker.x, path_blocker.y, path_blocker.yaw);
  path_blocker.valid = path_blocker.frenet.valid;
  ASSERT_TRUE(path_blocker.valid);
  collision_opponents.push_back(path_blocker);
  collision_guard.clearPassContinuationAuthorityPreservingSearchHint();
  const auto collision_output = collision_guard.update(
      ego527, collision_opponents, true, 32.204999280);
  EXPECT_NE(collision_output.mode, sl::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_FALSE(collision_guard.selected().has_value());
  EXPECT_EQ(collision_output.generated_candidates, 15);
  EXPECT_TRUE(collision_guard.lastPlanningCycleMetrics()
                  .accepted_path_continuation_evaluated);
  EXPECT_FALSE(collision_guard.lastPlanningCycleMetrics()
                   .accepted_path_continuation_accepted);

  // A fast connector that reaches the stop-cost boundary is provisional. It
  // must not suppress the ordinary 15-candidate search in the same update.
  sl::LatticePlanner stop_cost_fallback(planner);
  stop_cost_fallback.clearPassContinuationAuthorityPreservingSearchHint();
  ASSERT_TRUE(sl::StateLatticeTestAccess::hasAcceptedPathContinuationHint(
      stop_cost_fallback));
  sl::StateLatticeTestAccess::setStopCost(stop_cost_fallback, -1.0);
  const auto stop_cost_output = stop_cost_fallback.update(
      ego527, opponents527, true, 32.204999280);
  EXPECT_EQ(stop_cost_output.generated_candidates, 15);
  EXPECT_NE(stop_cost_output.mode, sl::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_FALSE(sl::StateLatticeTestAccess::hasAcceptedPathContinuationHint(
      stop_cost_fallback));
  EXPECT_EQ(stop_cost_fallback.lastPlanningCycleMetrics()
                .accepted_path_continuation_stage,
            "post_fast_path_stop_cost_rejected");
}

TEST(LatticePlanner,
     CampaignV26Gen471UsesExtendedAcceptedPathConnectorAfterLegacyMiss) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  auto config = campaignTrackabilityConfig(planning_frame);
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.2, -2.2};
  ASSERT_DOUBLE_EQ(config.planning_deadline_ms, 80.0);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const CampaignTrackabilityEgoInput input471{
      29.304999344, 89631.22327566595, 43129.03285132789,
      0.7873801816335906, 0.6164677198123628, 0.12023515232032049,
      -0.012629825717702957};
  const std::array<CampaignTrackabilityOpponentInput, 3U> opponents471_raw{{
      {"d2", 29.199999347, 89628.9140625, 43131.0, 29.249999346,
       89628.9140625, 43131.0},
      {"d3", 29.199999347, 89624.7265625, 43137.74609375,
       29.249999346, 89624.7265625, 43137.74609375},
      {"d4", 29.199999347, 89620.234375, 43144.671875,
       29.249999346, 89620.234375, 43144.671875},
  }};
  const auto ego471 = campaignTrackabilityEgo(input471, planning_frame);
  const auto opponents471 =
      campaignTrackabilityOpponents(opponents471_raw, planning_frame);

  std::vector<sl::TrajectoryPoint> accepted470;
  accepted470.reserve(v26_gen470_fixture::kV26Gen470Plan.size());
  for (const auto &recorded : v26_gen470_fixture::kV26Gen470Plan) {
    sl::TrajectoryPoint point;
    point.x = recorded.x;
    point.y = recorded.y;
    point.yaw = recorded.yaw;
    point.kappa = recorded.kappa;
    accepted470.push_back(point);
  }

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedAcceptedPathContinuationHint(
      planner, accepted470, "d2", -1, 2U, 1U, -1.0, 29.149999348);

  const auto generation471 =
      planner.update(ego471, opponents471, true, 29.319999344);
  const auto &diagnostic = planner.lastPlanningCycleMetrics();
  RecordProperty("gen471_connector_attempts",
                 diagnostic.accepted_path_continuation_connector_attempts);
  RecordProperty("gen471_connector_evaluator_rejects",
                 diagnostic.accepted_path_continuation_evaluator_rejects);
  RecordProperty("gen471_connector_stage",
                 diagnostic.accepted_path_continuation_stage);
  RecordProperty("gen471_connector_last_reject",
                 diagnostic.accepted_path_continuation_last_reject_reason);
  EXPECT_EQ(generation471.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation471.reason;
  EXPECT_EQ(generation471.feasible_candidates, 1);
  EXPECT_TRUE(diagnostic.accepted_path_continuation_requested);
  EXPECT_TRUE(diagnostic.accepted_path_continuation_exact_binding_valid);
  EXPECT_TRUE(diagnostic.accepted_path_continuation_anchor_valid);
  EXPECT_GT(diagnostic.accepted_path_continuation_connector_attempts, 48U);
  EXPECT_LE(diagnostic.accepted_path_continuation_connector_attempts, 73U);
  EXPECT_LT(diagnostic.accepted_path_continuation_evaluator_rejects,
            diagnostic.accepted_path_continuation_connector_attempts);
  EXPECT_TRUE(diagnostic.accepted_path_continuation_accepted);
  EXPECT_EQ(diagnostic.accepted_path_continuation_stage, "accepted");

  sl::LatticePlanner sweep_planner(config, &planning_frame, &map,
                                   &output_frame);
  sl::StateLatticeTestAccess::seedAcceptedPathContinuationHint(
      sweep_planner, accepted470, "d2", -1, 2U, 1U, -1.0, 29.149999348);
  const auto sweep =
      sl::StateLatticeTestAccess::sweepAcceptedPathConnectors(
          sweep_planner, ego471, opponents471, 0.60, 1.40, 0.005);
  RecordProperty("gen471_dense_sweep_eligible_join_points",
                 sweep.eligible_join_points);
  RecordProperty("gen471_dense_sweep_attempts", sweep.attempts);
  RecordProperty("gen471_dense_sweep_generation_rejects",
                 sweep.generation_rejects);
  RecordProperty("gen471_dense_sweep_evaluator_rejects",
                 sweep.evaluator_rejects);
  RecordProperty("gen471_dense_sweep_cartesian_rejects",
                 sweep.cartesian_rejects);
  RecordProperty("gen471_dense_sweep_accepted_join_index",
                 sweep.accepted_join_index);
  RecordProperty("gen471_dense_sweep_accepted_forward_arc_m",
                 sweep.accepted_forward_arc_m);
  RecordProperty("gen471_dense_sweep_accepted_tangent_scale",
                 sweep.accepted_tangent_scale);
  EXPECT_FALSE(sweep.accepted.has_value());

  const auto extended_sweep =
      sl::StateLatticeTestAccess::sweepAcceptedPathConnectors(
          sweep_planner, ego471, opponents471, 0.40, 1.60, 0.005, 4.00);
  RecordProperty("gen471_extended_sweep_eligible_join_points",
                 extended_sweep.eligible_join_points);
  RecordProperty("gen471_extended_sweep_attempts", extended_sweep.attempts);
  RecordProperty("gen471_extended_sweep_generation_rejects",
                 extended_sweep.generation_rejects);
  RecordProperty("gen471_extended_sweep_evaluator_rejects",
                 extended_sweep.evaluator_rejects);
  RecordProperty("gen471_extended_sweep_cartesian_rejects",
                 extended_sweep.cartesian_rejects);
  RecordProperty("gen471_extended_sweep_accepted_join_index",
                 extended_sweep.accepted_join_index);
  RecordProperty("gen471_extended_sweep_accepted_forward_arc_m",
                 std::to_string(extended_sweep.accepted_forward_arc_m));
  RecordProperty("gen471_extended_sweep_accepted_tangent_scale",
                 std::to_string(extended_sweep.accepted_tangent_scale));
  ASSERT_TRUE(extended_sweep.accepted.has_value())
      << "No unchanged-evaluator connector was found even after extending "
         "only the diagnostic connector horizon to 4m";
  EXPECT_TRUE(extended_sweep.accepted->feasible);
  EXPECT_GT(extended_sweep.accepted_forward_arc_m, 1.00);
  EXPECT_LE(extended_sweep.accepted_forward_arc_m, 4.00);
}

TEST(LatticePlanner,
     CampaignV29DiagnosesGen424To425AcceptedPathConnectorHorizon) {
  sl::FrenetFrame planning_frame;
  sl::FrenetFrame output_frame;
  std::string error;
  ASSERT_TRUE(planning_frame.loadCsv(
      std::string(TEST_STATE_LATTICE_SOURCE_DIR) +
          "/data/course_centerline.csv",
      &error)) << error;
  ASSERT_TRUE(output_frame.loadCsv(std::string(TEST_MPC_SOURCE_DIR) +
                                       "/env/final_ver3/traj_mincurv_manual.csv",
                                   &error)) << error;
  auto config = campaignTrackabilityConfig(planning_frame);
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.2, -2.2};
  ASSERT_DOUBLE_EQ(config.planning_deadline_ms, 80.0);
  ASSERT_DOUBLE_EQ(config.max_steer_rate_radps, 0.5);
  sl::GridMap map;
  ASSERT_TRUE(map.load(std::string(TEST_MPC_SOURCE_DIR) +
                           "/env/final_ver3/occupancy_grid_map.yaml",
                       config, &error)) << error;
  ASSERT_TRUE(map.buildReferenceLayer(planning_frame, config));

  const CampaignTrackabilityEgoInput input425{
      27.864999377, 89631.16294443305, 43128.866184844686,
      0.8016729829532839, 0.5977628529799955, 0.09503430398332537,
      -0.0019509242552646164};
  const std::array<CampaignTrackabilityOpponentInput, 3U> opponents425_raw{{
      {"d2", 27.799999378, 89628.9140625, 43131.0, 27.849999377,
       89628.9140625, 43131.0},
      {"d3", 27.799999378, 89624.734375, 43137.75, 27.849999377,
       89624.734375, 43137.75},
      {"d4", 27.799999378, 89620.234375, 43144.671875, 27.849999377,
       89620.234375, 43144.671875},
  }};
  const auto ego425 = campaignTrackabilityEgo(input425, planning_frame);
  const auto opponents425 =
      campaignTrackabilityOpponents(opponents425_raw, planning_frame);

  std::vector<sl::TrajectoryPoint> accepted424;
  accepted424.reserve(v29_gen424_fixture::kV29Gen424Plan.size());
  for (const auto &recorded : v29_gen424_fixture::kV29Gen424Plan) {
    sl::TrajectoryPoint point;
    point.x = recorded.x;
    point.y = recorded.y;
    point.yaw = recorded.yaw;
    point.kappa = recorded.kappa;
    accepted424.push_back(point);
  }

  sl::LatticePlanner planner(config, &planning_frame, &map, &output_frame);
  sl::StateLatticeTestAccess::seedAcceptedPathContinuationHint(
      planner, accepted424, "d2", -1, 2U, 1U, -1.0, 27.7, 1.03);
  const auto generation425_started = std::chrono::steady_clock::now();
  const auto generation425 =
      planner.update(ego425, opponents425, true, 27.944999375);
  const double generation425_elapsed_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - generation425_started)
          .count();
  const auto &diagnostic = planner.lastPlanningCycleMetrics();
  RecordProperty("gen425_production_update_elapsed_ms",
                 generation425_elapsed_ms);
  RecordProperty("gen425_connector_attempts",
                 diagnostic.accepted_path_continuation_connector_attempts);
  RecordProperty("gen425_connector_evaluator_rejects",
                 diagnostic.accepted_path_continuation_evaluator_rejects);
  RecordProperty("gen425_connector_stage",
                 diagnostic.accepted_path_continuation_stage);
  RecordProperty("gen425_connector_last_reject",
                 diagnostic.accepted_path_continuation_last_reject_reason);
  EXPECT_EQ(generation425.mode, sl::BehaviorMode::OVERTAKE_RIGHT)
      << generation425.reason;
  EXPECT_EQ(generation425.feasible_candidates, 1);
  EXPECT_TRUE(diagnostic.accepted_path_continuation_requested);
  EXPECT_TRUE(diagnostic.accepted_path_continuation_exact_binding_valid);
  EXPECT_TRUE(diagnostic.accepted_path_continuation_anchor_valid);
  EXPECT_GT(diagnostic.accepted_path_continuation_connector_attempts, 48U);
  EXPECT_LE(diagnostic.accepted_path_continuation_connector_attempts, 73U);
  EXPECT_TRUE(diagnostic.accepted_path_continuation_accepted);
  EXPECT_EQ(diagnostic.accepted_path_continuation_stage, "accepted");
  EXPECT_LT(generation425_elapsed_ms, config.planning_deadline_ms);

  sl::LatticePlanner sweep_planner(config, &planning_frame, &map,
                                   &output_frame);
  sl::StateLatticeTestAccess::seedAcceptedPathContinuationHint(
      sweep_planner, accepted424, "d2", -1, 2U, 1U, -1.0, 27.7, 1.03);
  const auto one_metre_sweep =
      sl::StateLatticeTestAccess::sweepAcceptedPathConnectors(
          sweep_planner, ego425, opponents425, 0.40, 1.60, 0.005, 1.00);
  RecordProperty("gen425_one_metre_attempts", one_metre_sweep.attempts);
  RecordProperty("gen425_one_metre_accepted_join_index",
                 one_metre_sweep.accepted_join_index);
  EXPECT_FALSE(one_metre_sweep.accepted.has_value());

  const auto extended_sweep =
      sl::StateLatticeTestAccess::sweepAcceptedPathConnectors(
          sweep_planner, ego425, opponents425, 0.40, 1.60, 0.005, 4.00);
  RecordProperty("gen425_extended_sweep_eligible_join_points",
                 extended_sweep.eligible_join_points);
  RecordProperty("gen425_extended_sweep_attempts", extended_sweep.attempts);
  RecordProperty("gen425_extended_sweep_evaluator_rejects",
                 extended_sweep.evaluator_rejects);
  RecordProperty("gen425_extended_sweep_accepted_join_index",
                 extended_sweep.accepted_join_index);
  RecordProperty("gen425_extended_sweep_accepted_forward_arc_m",
                 std::to_string(extended_sweep.accepted_forward_arc_m));
  RecordProperty("gen425_extended_sweep_accepted_tangent_scale",
                 std::to_string(extended_sweep.accepted_tangent_scale));
  ASSERT_TRUE(extended_sweep.accepted.has_value())
      << "No unchanged-evaluator connector was found in the diagnostic 4m "
         "horizon";
  EXPECT_TRUE(extended_sweep.accepted->feasible);
  EXPECT_GT(extended_sweep.accepted_forward_arc_m, 1.00);
  EXPECT_LE(extended_sweep.accepted_forward_arc_m, 4.00);
}
