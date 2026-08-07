#pragma once

#include "simple_state_lattice_planner/types.hpp"

#include <cstdint>
#include <string>

namespace simple_state_lattice_planner {

struct InstantControllerConfig {
  double wheel_base_m{1.087};
  double lookahead_min_m{1.5};
  double lookahead_gain_sec{0.35};
  double maximum_steering_angle_rad{0.5236};
  double maximum_steering_rate_radps{0.35};
  double speed_proportional_gain{0.8};
  double minimum_acceleration_mps2{-0.9};
  double maximum_acceleration_mps2{3.0};
  double maximum_dt_sec{0.20};
};

struct InstantControlInput {
  const EgoState *ego{nullptr};
  const Candidate *selected_candidate{nullptr};
  std::uint64_t planner_snapshot_id{0U};
  bool inputs_fresh{false};
  bool planner_output_fresh{false};
  std::int64_t monotonic_now_ns{0};
};

struct InstantControlResult {
  bool valid{false};
  bool stop_required{true};
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
  double steering_angle_rad{0.0};
  double steering_rate_radps{0.0};
  std::string reason{"uninitialized"};
};

class InstantController {
 public:
  explicit InstantController(
      InstantControllerConfig config = InstantControllerConfig{});
  InstantControlResult update(const InstantControlInput &input);
  void reset();

 private:
  InstantControlResult invalid(const char *reason);

  InstantControllerConfig config_;
  bool initialized_{false};
  double last_steering_rad_{0.0};
  std::int64_t last_monotonic_ns_{0};
};

}  // namespace simple_state_lattice_planner
