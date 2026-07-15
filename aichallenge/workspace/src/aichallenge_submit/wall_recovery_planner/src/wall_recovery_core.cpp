#include "wall_recovery_planner/wall_recovery_core.hpp"

#include <algorithm>
#include <cmath>

namespace wall_recovery_planner {

double normalizeAngle(double angle_rad) {
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

ForwardWaypointResult selectForwardWaypoint(
    const std::vector<Waypoint2d> &trajectory, double ego_x, double ego_y,
    double ego_yaw, const ForwardWaypointConfig &config) {
  ForwardWaypointResult result;
  if (trajectory.empty()) {
    result.reason = "empty_trajectory";
    return result;
  }
  if (!std::isfinite(ego_x) || !std::isfinite(ego_y) ||
      !std::isfinite(ego_yaw)) {
    result.reason = "invalid_ego_pose";
    return result;
  }

  const double cos_yaw = std::cos(ego_yaw);
  const double sin_yaw = std::sin(ego_yaw);
  double arc_distance_m = 0.0;
  bool saw_candidate_arc = false;
  std::optional<Waypoint2d> previous;

  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const auto &point = trajectory.at(i);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.yaw)) {
      result.reason = "invalid_trajectory_value";
      return result;
    }

    if (previous.has_value()) {
      const double ds =
          std::hypot(point.x - previous->x, point.y - previous->y);
      if (ds > std::max(0.0, config.max_segment_length_m)) {
        result.reason = "trajectory_discontinuity";
        return result;
      }
      arc_distance_m += ds;
    } else {
      arc_distance_m = std::hypot(point.x - ego_x, point.y - ego_y);
    }
    previous = point;

    if (arc_distance_m < std::max(0.0, config.min_arc_distance_m)) {
      continue;
    }
    saw_candidate_arc = true;
    if (arc_distance_m > std::max(config.min_arc_distance_m,
                                  config.max_arc_distance_m)) {
      result.reason = "no_waypoint_within_arc_window";
      return result;
    }

    const double dx = point.x - ego_x;
    const double dy = point.y - ego_y;
    const double body_forward_dot = dx * cos_yaw + dy * sin_yaw;
    if (body_forward_dot <= 0.0) {
      result.reason = "candidate_behind_ego";
      continue;
    }

    const double heading_error = std::abs(normalizeAngle(point.yaw - ego_yaw));
    if (heading_error > std::max(0.0, config.max_heading_error_rad)) {
      result.reason = "heading_error";
      continue;
    }

    result.valid = true;
    result.index = i;
    result.arc_distance_m = arc_distance_m;
    result.reason = "selected";
    return result;
  }

  result.reason = saw_candidate_arc ? result.reason : "trajectory_too_short";
  return result;
}

std::vector<Waypoint2d> buildRecoveryTrajectory(
    double ego_x, double ego_y, double ego_yaw, const Waypoint2d &target,
    double velocity_mps, std::size_t point_count) {
  const std::size_t count = std::max<std::size_t>(2, point_count);
  std::vector<Waypoint2d> trajectory;
  trajectory.reserve(count);

  const double capped_velocity =
      std::isfinite(velocity_mps) ? std::max(0.0, velocity_mps) : 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const double ratio =
        count <= 1 ? 1.0 : static_cast<double>(i) / static_cast<double>(count - 1);
    Waypoint2d point;
    point.x = ego_x + (target.x - ego_x) * ratio;
    point.y = ego_y + (target.y - ego_y) * ratio;
    point.yaw = normalizeAngle(ego_yaw + normalizeAngle(target.yaw - ego_yaw) * ratio);
    point.velocity_mps = capped_velocity;
    trajectory.push_back(point);
  }
  return trajectory;
}

CollisionEventTracker::CollisionEventTracker(int edge_min_delta)
    : edge_min_delta_(std::max(1, edge_min_delta)) {}

CollisionEventResult CollisionEventTracker::observe(int value) {
  CollisionEventResult result;
  if (!has_last_) {
    has_last_ = true;
    last_value_ = value;
    result.sequence = sequence_;
    result.reason = "baseline";
    return result;
  }

  const int delta = value - last_value_;
  last_value_ = value;
  if (delta < 0) {
    result.sequence = sequence_;
    result.reason = "counter_reset";
    return result;
  }
  if (delta >= edge_min_delta_) {
    ++sequence_;
    result.event = true;
    result.sequence = sequence_;
    result.reason = "collision_edge";
    return result;
  }
  result.sequence = sequence_;
  result.reason = "delta_below_threshold";
  return result;
}

std::uint32_t CollisionEventTracker::sequence() const { return sequence_; }

}  // namespace wall_recovery_planner
