#ifndef SIMPLE_TRAJECTORY_GENERATOR__EXECUTION_PROFILE_HPP_
#define SIMPLE_TRAJECTORY_GENERATOR__EXECUTION_PROFILE_HPP_

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <cstddef>
#include <string>

namespace simple_trajectory_generator {

struct ExecutionProfileConfig {
  double max_speed_mps{10.0};
  double max_arc_spacing_m{0.25};
  double max_yaw_step_rad{0.05};
  std::size_t max_output_points{10000U};
};

struct ExecutionProfileResult {
  bool valid{false};
  std::string reason{};
  autoware_auto_planning_msgs::msg::Trajectory trajectory{};
  double execution_speed_mps{0.0};
};

ExecutionProfileResult buildExecutionProfile(
    const autoware_auto_planning_msgs::msg::Trajectory &source,
    const ExecutionProfileConfig &config);

} // namespace simple_trajectory_generator

#endif // SIMPLE_TRAJECTORY_GENERATOR__EXECUTION_PROFILE_HPP_
