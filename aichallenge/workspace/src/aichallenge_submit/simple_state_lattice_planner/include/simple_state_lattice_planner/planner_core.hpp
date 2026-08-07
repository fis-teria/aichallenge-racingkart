#pragma once

#include "simple_state_lattice_planner/candidate_evaluator.hpp"
#include "simple_state_lattice_planner/lattice_generator.hpp"

#include <optional>

namespace simple_state_lattice_planner {

enum class OutputReason {
  CENTER_CLEAR,
  CENTER_BLOCKED_SIDE_SELECTED,
  OVERTAKE_CONTINUING,
  OPPONENT_PASSED_CENTER_CLEAR,
  NO_VALID_CANDIDATE,
  INPUT_INVALID,
  OPPONENT_UNAVAILABLE,
};

struct PlannerConfig {
  LatticeConfig lattice;
  CandidateEvaluatorConfig evaluator;
  double pass_clearance_m{2.0};
};

struct PlannerInput {
  std::uint64_t snapshot_id{0U};
  EgoState ego;
  std::optional<OpponentState> opponent;
  ReferenceWindow reference;
  Costmap2D costmap;
};

struct PlannerOutput {
  std::uint64_t snapshot_id{0U};
  PlannerState state{PlannerState::FREE_RUN};
  std::vector<Candidate> candidates;
  std::optional<std::uint32_t> selected_id;
  std::vector<TrajectoryPoint> trajectory;
  double speed_limit_mps{0.0};
  OutputReason reason{OutputReason::INPUT_INVALID};
};

class PlannerCore {
 public:
  explicit PlannerCore(PlannerConfig config = PlannerConfig{});
  PlannerOutput plan(const PlannerInput &input, double snapshot_time_sec);
  void reset();

 private:
  PlannerConfig config_;
  PlannerState state_{PlannerState::FREE_RUN};
  std::optional<CandidateSide> selected_side_;
};

}  // namespace simple_state_lattice_planner
