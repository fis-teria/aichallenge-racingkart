#pragma once

#include "state_lattice_overtake_planner/types.hpp"

#include <string>
#include <vector>

namespace state_lattice_overtake_planner {

struct InstantControllerConfig {
  double wheel_base_m{1.087};
  double lookahead_min_m{1.5};
  double lookahead_gain_sec{0.35};
  double maximum_steering_angle_rad{0.5236};
  double maximum_steering_rate_radps{0.35};
  double speed_proportional_gain{0.8};
  double minimum_acceleration_mps2{-0.9};
  double maximum_acceleration_mps2{3.0};
};

struct InstantControlResult {
  bool valid{false};
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
  double steering_angle_rad{0.0};
  double steering_rate_radps{0.0};
  std::string reason{"uninitialized"};
};

class InstantController {
public:
  explicit InstantController(InstantControllerConfig config);

  InstantControlResult update(const EgoState &ego,
                              const std::vector<TrajectoryPoint> &trajectory,
                              double now_sec);
  void reset();

private:
  InstantControllerConfig config_;
  bool initialized_{false};
  double last_steering_rad_{0.0};
  double last_time_sec_{0.0};
};

} // namespace state_lattice_overtake_planner
