#pragma once

#include <array>
#include <string_view>

namespace state_lattice_overtake_planner::test_fixture {

struct PoseSnapshot {
  std::string_view vehicle_id;
  double x_m;
  double y_m;
  double yaw_rad;
  double speed_mps;
};

struct CurrentClearanceSnapshot {
  PoseSnapshot ego;
  PoseSnapshot opponent;
  double opponent_uncertainty_x_m;
  double opponent_uncertainty_y_m;
  double expected_clearance_m;
  int generation;
};

// Extracted from output/20260727-225451 at the first
// current_pose_hard_collision for each vehicle. Raw bags stay out of source.
inline constexpr std::array<CurrentClearanceSnapshot, 3> kCurrentClearance{{
    {
        {"d1", 89629.21517932293, 43133.607259596836, 2.05624548876371,
         2.8407670556123072},
        {"d2", 89628.1953125, 43136.08203125, 2.408777551803287,
         1.0512714978497505},
        0.15,
        0.15,
        0.11509451652183174,
        899,
    },
    {
        {"d2", 89629.15770704675, 43135.53524190416, 2.4402467245210517,
         3.021509934460926},
        {"d3", 89626.3984375, 43136.015625, 2.0183163019520665,
         2.166460701860923},
        0.15,
        0.15,
        0.20932342020912872,
        4,
    },
    {
        {"d3", 89627.05912687065, 43135.17168791852, 2.0280313536605736,
         2.225219466851543},
        {"d2", 89629.7578125, 43134.8046875, 2.453923218437667,
         2.830574519210099},
        0.15,
        0.15,
        0.17693498850834077,
        4,
    },
}};

struct CandidateSnapshot {
  PoseSnapshot ego;
  std::array<PoseSnapshot, 2> opponents;
  double opponent_uncertainty_x_m;
  double opponent_uncertainty_y_m;
  int observed_generated_candidates;
  int observed_feasible_candidates;
  int expected_wall_rejections;
  int expected_opponent_rejections;
  int expected_curvature_rejections;
  int expected_trackability_rejections;
  int expected_other_rejections;
  int generation;
};

// D1's first fully joined FOLLOW_BLOCKED sample in the same run. D2 has the
// smaller Frenet-s gap, but D3 is closer to D1's transition corridor. The
// start-grid target selector must therefore keep D3 as the longitudinal
// blocker while D2 remains in every candidate's all-opponent safety check.
inline constexpr CandidateSnapshot kD1FollowBlocked{
    {"d1", 89631.41621030391, 43127.808466350005, 2.214920786016026, 0.0},
    {{
        {"d2", 89633.046875, 43131.17578125, 0.0, 0.0},
        {"d3", 89628.8046875, 43131.41796875, 0.0, 0.0},
    }},
    0.15,
    0.15,
    21,
    0,
    3,
    0,
    0,
    18,
    0,
    28,
};

struct PreHardParallelSnapshot {
  PoseSnapshot ego;
  std::array<PoseSnapshot, 2> opponents;
  double opponent_uncertainty_x_m;
  double opponent_uncertainty_y_m;
  std::string_view expected_target_id;
};

// D3's final FREE_RUN sample before current_pose_hard_collision in
// output/20260727-233929. D2 is inside the physical detection envelope but was
// not admitted by the old forward-only Frenet test.
inline constexpr PreHardParallelSnapshot kD3PreHardParallel{
    {"d3", 89627.25858120657, 43134.76143526465, 2.03216, 2.1020469002641335},
    {{
        {"d1", 89629.9921875, 43131.1328125, 0.0, 0.0},
        {"d2", 89629.9921875, 43134.6171875, 0.0, 0.0},
    }},
    0.15,
    0.15,
    "d2",
};

} // namespace state_lattice_overtake_planner::test_fixture
