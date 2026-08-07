#include "overtake_planner/cartesian_trackability_evaluator.hpp"
#include "overtake_planner/state_lattice_shadow_adapter.hpp"

#include <gtest/gtest.h>

namespace overtake_planner {
namespace {
FrenetFrame makeStraightFrame() {
  FrenetFrame frame;
  std::vector<ReferencePoint> reference;
  for (int index = 0; index < 100; ++index) {
    reference.push_back({static_cast<double>(index), static_cast<double>(index),
                         0.0, 0.0, 0.0, 5.0});
  }
  frame.setReference(std::move(reference));
  return frame;
}
} // namespace

TEST(StateLatticeShadowAdapter, ReprojectsDenseGeometryAsUnevaluatedCandidate) {
  auto frame = makeStraightFrame();
  state_lattice_overtake_planner::ShadowGeometryRequest request;
  request.start = {1.0, 0.0, 0.0};
  request.goal = {8.0, 1.0, 0.0};
  request.target_id = "D2";
  request.pass_side = 1;
  request.target_d_m = 1.0;
  request.sample_count = 41U;
  const auto geometry =
      state_lattice_overtake_planner::ShadowGeometryGenerator{}.generate(
          request);
  CandidateTrajectory current;
  for (std::size_t index = 0U; index < geometry.dense.size(); ++index) {
    current.t.push_back(0.025 * static_cast<double>(index));
    current.longitudinal_offsets_m.push_back(
        7.0 * static_cast<double>(index) /
        static_cast<double>(geometry.dense.size() - 1U));
    current.predicted_speed_mps.push_back(2.0 +
                                          0.1 * static_cast<double>(index));
    current.v_ref.push_back(3.0);
  }
  const auto adapted = StateLatticeShadowAdapter(frame).adapt(
      geometry, current, CandidateType::PASS_LEFT, "D2", 1, 1.0);
  ASSERT_TRUE(adapted.valid) << adapted.reason;
  EXPECT_FALSE(adapted.candidate.safety_evaluated);
  EXPECT_FALSE(adapted.candidate.feasible);
  EXPECT_FALSE(adapted.candidate.desired_path_trackable);
  EXPECT_FALSE(adapted.candidate.pure_pursuit_command_trackable);
  EXPECT_EQ(adapted.candidate.s.size(), geometry.dense.size());
  EXPECT_GT(adapted.candidate.longitudinal_offsets_m.back(), 0.0);
  EXPECT_EQ(adapted.candidate.longitudinal_offsets_m,
            current.longitudinal_offsets_m);
  EXPECT_EQ(adapted.candidate.t, current.t);
  EXPECT_EQ(adapted.candidate.v_ref, current.v_ref);
}

TEST(StateLatticeShadowAdapter, FailsClosedOnInvalidGeometry) {
  auto frame = makeStraightFrame();
  state_lattice_overtake_planner::ShadowGeometryResult geometry;
  geometry.valid = true;
  CandidateTrajectory current;
  const auto adapted = StateLatticeShadowAdapter(frame).adapt(
      geometry, current, CandidateType::PASS_LEFT, "D2", 1, 1.0);
  EXPECT_FALSE(adapted.valid);
}

TEST(StateLatticeShadowAdapter,
     AcceptsValidNonCanonicalAsUnevaluatedAndExactEvaluatorRejects) {
  auto frame = makeStraightFrame();
  state_lattice_overtake_planner::ShadowGeometryRequest request;
  request.start = {1.0, 0.0, 0.0};
  request.goal = {8.0, 1.0, 0.0};
  request.target_id = "D2";
  request.pass_side = 1;
  request.target_d_m = 1.0;
  request.sample_count = 41U;
  auto geometry =
      state_lattice_overtake_planner::ShadowGeometryGenerator{}.generate(
          request);
  ASSERT_TRUE(geometry.valid);
  geometry.dense[geometry.dense.size() / 2U].y += 0.25;
  CandidateTrajectory current;
  for (std::size_t index = 0U; index < geometry.dense.size(); ++index) {
    current.t.push_back(0.025 * static_cast<double>(index));
    current.longitudinal_offsets_m.push_back(
        7.0 * static_cast<double>(index) /
        static_cast<double>(geometry.dense.size() - 1U));
    current.predicted_speed_mps.push_back(2.0);
    current.v_ref.push_back(3.0);
  }
  const auto adapted = StateLatticeShadowAdapter(frame).adapt(
      geometry, current, CandidateType::PASS_LEFT, "D2", 1, 1.0);
  ASSERT_TRUE(adapted.valid) << adapted.reason;
  EXPECT_FALSE(adapted.candidate.safety_evaluated);
  EXPECT_FALSE(adapted.candidate.feasible);

  CartesianTrackabilityConfig config;
  config.pure_pursuit_required_arc_m = 0.5;
  config.target_d_m = 1.0;
  config.target_d_deadline_arc_m = 7.5;
  CartesianTrackabilityInput input;
  input.ego.valid = true;
  input.ego.x = adapted.candidate.x.front();
  input.ego.y = adapted.candidate.y.front();
  input.ego.yaw = adapted.candidate.yaw.front();
  input.ego.v = 2.0;
  input.active_lookahead_valid = true;
  input.active_lookahead_distance_m = 1.0;
  input.nearest_source_index_valid = true;
  input.nearest_source_index = 0U;
  input.curvature_feedforward_valid = true;
  input.curvature_feedforward_steering_rad = 0.0;
  input.steering_reference_valid = true;
  input.steering_reference_angle_rad = 0.0;
  input.steering_command_dt_sec = 0.01;
  const auto exact = CartesianTrackabilityEvaluator(frame).evaluate(
      adapted.candidate, input, config);
  EXPECT_FALSE(exact.valid && exact.trackable);
  EXPECT_NE(exact.reason, "ok");
}
} // namespace overtake_planner
