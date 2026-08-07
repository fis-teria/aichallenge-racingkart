#pragma once

#include <array>
#include <string_view>

namespace state_lattice_overtake_planner::test_fixture {

struct OutputContractPoseSnapshot {
  std::string_view vehicle_id;
  double x_m;
  double y_m;
  double yaw_rad;
  double speed_mps;
};

struct OutputContractSnapshot {
  OutputContractPoseSnapshot ego;
  std::array<OutputContractPoseSnapshot, 2> opponents;
  double opponent_uncertainty_x_m;
  double opponent_uncertainty_y_m;
  int observed_generated_candidates;
  int bag_observed_feasible_candidates;
  int replay_expected_feasible_candidates;
  std::string_view observed_reason;
};

// Extracted from output/20260728-010706/d2 at clock=1.904999957, the first
// follow_blocked_output_contract sample. The live planner generated 21
// candidates, accepted 16 geometrically, then rejected every resampled output
// horizon.
inline constexpr OutputContractSnapshot kD2FirstOutputContract{
    {"d2", 89633.30679316094, 43131.17568487456, 2.216058185742633, 0.0},
    {{
        {"d1", 89631.15625, 43127.80859375, 0.0, 0.0},
        {"d3", 89628.8046875, 43131.41796875, 0.0, 0.0},
    }},
    0.15,
    0.15,
    21,
    16,
    19,
    "follow_blocked_output_contract",
};

} // namespace state_lattice_overtake_planner::test_fixture
