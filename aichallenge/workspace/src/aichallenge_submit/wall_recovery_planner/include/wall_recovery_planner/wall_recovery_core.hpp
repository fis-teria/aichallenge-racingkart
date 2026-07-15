#ifndef WALL_RECOVERY_PLANNER__WALL_RECOVERY_CORE_HPP_
#define WALL_RECOVERY_PLANNER__WALL_RECOVERY_CORE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace wall_recovery_planner {

struct Waypoint2d {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double velocity_mps{0.0};
};

struct ForwardWaypointConfig {
  double min_arc_distance_m{1.0};
  double max_arc_distance_m{5.0};
  double max_heading_error_rad{1.2};
  double max_segment_length_m{5.0};
};

struct ForwardWaypointResult {
  bool valid{false};
  std::size_t index{0};
  double arc_distance_m{0.0};
  std::string reason{"not_evaluated"};
};

struct CollisionEventResult {
  bool event{false};
  std::uint32_t sequence{0};
  std::string reason{"no_event"};
};

double normalizeAngle(double angle_rad);

ForwardWaypointResult selectForwardWaypoint(
    const std::vector<Waypoint2d> &trajectory, double ego_x, double ego_y,
    double ego_yaw, const ForwardWaypointConfig &config);

std::vector<Waypoint2d> buildRecoveryTrajectory(
    double ego_x, double ego_y, double ego_yaw, const Waypoint2d &target,
    double velocity_mps, std::size_t point_count);

class CollisionEventTracker {
public:
  explicit CollisionEventTracker(int edge_min_delta);

  CollisionEventResult observe(int value);
  std::uint32_t sequence() const;

private:
  int edge_min_delta_{30};
  bool has_last_{false};
  int last_value_{0};
  std::uint32_t sequence_{0};
};

}  // namespace wall_recovery_planner

#endif  // WALL_RECOVERY_PLANNER__WALL_RECOVERY_CORE_HPP_
