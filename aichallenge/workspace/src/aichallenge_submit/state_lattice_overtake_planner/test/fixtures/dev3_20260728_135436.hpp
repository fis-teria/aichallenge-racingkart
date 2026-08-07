#pragma once

#include <array>
#include <string_view>

namespace state_lattice_overtake_planner::test_fixture {

struct D2TrackabilityPoseSnapshot {
  std::string_view vehicle_id;
  double x_m;
  double y_m;
  double yaw_rad;
  double speed_mps;
};

struct D2TrackabilitySnapshot {
  D2TrackabilityPoseSnapshot ego;
  std::array<D2TrackabilityPoseSnapshot, 2> opponents;
  double opponent_uncertainty_x_m;
  double opponent_uncertainty_y_m;
  int observed_generated_candidates;
  int bag_observed_feasible_candidates;
  int replay_expected_feasible_candidates;
  int observed_wall_rejections;
  int observed_opponent_rejections;
  int bag_observed_trackability_rejections;
  int replay_expected_trackability_rejections;
};

// Extracted from output/20260728-135436/d2 at the first metrics sample with a
// trackability rejection (bag stamp 1785214554504909976). Ego and V2X values
// are the most recent messages preceding that sample; no simulator truth topic
// is used. The single-snapshot replay lacks the live opponent-history state and
// therefore preserves both the bag aggregate and the current replay aggregate
// explicitly instead of pretending they are identical.
inline constexpr D2TrackabilitySnapshot kD2FirstTrackabilityReject{
    {"d2", 89632.38422404173, 43132.319150365096, 2.413647963592652,
     1.4100874980578766},
    {{
        {"d1", 89631.015625, 43127.97265625, 0.0, 0.0},
        {"d3", 89627.4296875, 43133.921875, 0.0, 0.0},
    }},
    0.15,
    0.15,
    15,
    2,
    3,
    6,
    6,
    1,
    0,
};

} // namespace state_lattice_overtake_planner::test_fixture
