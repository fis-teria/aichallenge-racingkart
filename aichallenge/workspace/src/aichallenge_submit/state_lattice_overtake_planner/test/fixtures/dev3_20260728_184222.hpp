#pragma once

#include <array>
#include <string_view>

namespace state_lattice_overtake_planner::test_fixture {

struct StartTargetPoseSnapshot {
  std::string_view vehicle_id;
  double x_m;
  double y_m;
  double yaw_rad;
  double speed_mps;
};

// Extracted from D1's first joined /v2x/vehicle_positions and
// /debug/overtake/metrics sample in output/20260728-184222. The raw bag is
// intentionally not part of the source tree.
inline constexpr StartTargetPoseSnapshot kD1StartTargetEgo{
    "d1", 89631.41621030391, 43127.808466350005, 2.214920786016026, 0.0};

inline constexpr std::array<StartTargetPoseSnapshot, 2> kD1StartTargetOpponents{
    {
        {"d2", 89633.046875, 43131.17578125, 0.0, 0.0},
        {"d3", 89628.8046875, 43131.41796875, 0.0, 0.0},
    }};

} // namespace state_lattice_overtake_planner::test_fixture
