#pragma once

#include "simple_state_lattice_planner/costmap_builder.hpp"
#include "simple_state_lattice_planner/types.hpp"

namespace simple_state_lattice_planner {

struct CandidateEvaluatorConfig {
  double wheel_base_m{1.087};
  double maximum_curvature_radpm{0.50};
  double maximum_steering_angle_rad{0.5236};
  double maximum_steering_rate_radps{0.35};
  double minimum_rate_evaluation_speed_mps{0.1};
  double maximum_segment_length_m{0.05};
  double obstacle_cost_weight{1.0};
  double reference_deviation_weight{1.0};
  double curvature_weight{1.0};
  double steering_change_weight{1.0};
};

Candidate evaluateCandidate(
    Candidate candidate, const Costmap2D &costmap,
    const CandidateEvaluatorConfig &config = CandidateEvaluatorConfig{});

}  // namespace simple_state_lattice_planner
