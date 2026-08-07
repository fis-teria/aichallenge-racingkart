#pragma once

#include <array>
#include <string_view>

namespace state_lattice_overtake_planner::test_fixture {

struct Dev3PoseSnapshot {
  std::string_view vehicle_id;
  double x_m;
  double y_m;
  double yaw_rad;
  double speed_mps;
};

// Extracted from the joined localization, V2X and planner-debug samples in
// output/20260728-213709. The raw bag remains a generated artifact.
inline constexpr Dev3PoseSnapshot kD1StartEgo{
    "d1", 89631.41619815232, 43127.808519545855, 2.216047090471261,
    -2.0312073437745432e-06};

inline constexpr std::array<Dev3PoseSnapshot, 2> kD1StartOpponents{{
    {"d2", 89633.046875, 43131.17578125, 0.0, 0.0},
    {"d3", 89628.8046875, 43131.41796875, 0.0, 0.0},
}};

// First D1 transition after the start-grid front target is measurably moving.
inline constexpr Dev3PoseSnapshot kD1MovingReleaseEgo{
    "d1", 89631.06990653773, 43128.35782920393, 2.05698693632992,
    0.7781318519827356};

inline constexpr std::array<Dev3PoseSnapshot, 2> kD1MovingReleaseOpponents{{
    {"d2", 89632.96875, 43131.30859375, 2.17, 0.40},
    {"d3", 89628.15625, 43132.4921875, 2.08, 1.35},
}};

inline constexpr Dev3PoseSnapshot kD2CostStopEgo{"d2", 89633.156, 43131.412,
                                                 2.172, 0.2172617708323193};

inline constexpr std::array<Dev3PoseSnapshot, 2> kD2CostStopOpponents{{
    {"d1", 89630.609375, 43128.86328125, 1.83139871854, 1.21282617240},
    {"d3", 89627.7578125, 43133.19921875, 2.07789483119, 1.60869224170},
}};

} // namespace state_lattice_overtake_planner::test_fixture
