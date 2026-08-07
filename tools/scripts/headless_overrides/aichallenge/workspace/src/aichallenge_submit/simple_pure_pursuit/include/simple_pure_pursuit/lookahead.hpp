#ifndef SIMPLE_PURE_PURSUIT_LOOKAHEAD_HPP_
#define SIMPLE_PURE_PURSUIT_LOOKAHEAD_HPP_

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace simple_pure_pursuit {

struct LookaheadParams {
  double lookahead_gain{1.0};
  double lookahead_min_distance{1.0};
  bool curvature_adaptive_enabled{false};
  double curvature_min_distance{1.0};
  double curvature_sensitivity{0.0};
};

inline double
distance2d(const autoware_auto_planning_msgs::msg::TrajectoryPoint &a,
           const autoware_auto_planning_msgs::msg::TrajectoryPoint &b) {
  return std::hypot(a.pose.position.x - b.pose.position.x,
                    a.pose.position.y - b.pose.position.y);
}

inline double effectiveLookaheadSpeed(double target_speed_mps,
                                      double current_speed_mps) {
  const double target_mps =
      std::isfinite(target_speed_mps) ? std::max(0.0, target_speed_mps) : 0.0;
  const double current_mps =
      std::isfinite(current_speed_mps) ? std::max(0.0, current_speed_mps) : 0.0;
  return std::max(target_mps, current_mps);
}

inline double speedBasedLookaheadDistance(double target_speed_mps,
                                          double current_speed_mps,
                                          const LookaheadParams &params) {
  const double min_distance_m =
      std::isfinite(params.lookahead_min_distance)
          ? std::max(0.0, params.lookahead_min_distance)
          : 0.0;
  const double gain = std::isfinite(params.lookahead_gain)
                          ? std::max(0.0, params.lookahead_gain)
                          : 0.0;
  return gain * effectiveLookaheadSpeed(target_speed_mps, current_speed_mps) +
         min_distance_m;
}

inline double curvatureFromThreePoints(
    const autoware_auto_planning_msgs::msg::TrajectoryPoint &a,
    const autoware_auto_planning_msgs::msg::TrajectoryPoint &b,
    const autoware_auto_planning_msgs::msg::TrajectoryPoint &c) {
  const double ab = distance2d(a, b);
  const double bc = distance2d(b, c);
  const double ca = distance2d(c, a);
  const double denominator = ab * bc * ca;
  if (!std::isfinite(denominator) || denominator <= 1.0e-9) {
    return 0.0;
  }

  const double ab_x = b.pose.position.x - a.pose.position.x;
  const double ab_y = b.pose.position.y - a.pose.position.y;
  const double ac_x = c.pose.position.x - a.pose.position.x;
  const double ac_y = c.pose.position.y - a.pose.position.y;
  const double twice_area = std::abs(ab_x * ac_y - ab_y * ac_x);
  if (!std::isfinite(twice_area)) {
    return 0.0;
  }
  return 2.0 * twice_area / denominator;
}

inline double signedCurvatureFromThreePoints(
    const autoware_auto_planning_msgs::msg::TrajectoryPoint &a,
    const autoware_auto_planning_msgs::msg::TrajectoryPoint &b,
    const autoware_auto_planning_msgs::msg::TrajectoryPoint &c) {
  const double ab = distance2d(a, b);
  const double bc = distance2d(b, c);
  const double ca = distance2d(c, a);
  const double denominator = ab * bc * ca;
  if (!std::isfinite(denominator) || denominator <= 1.0e-9) {
    return 0.0;
  }

  const double ab_x = b.pose.position.x - a.pose.position.x;
  const double ab_y = b.pose.position.y - a.pose.position.y;
  const double ac_x = c.pose.position.x - a.pose.position.x;
  const double ac_y = c.pose.position.y - a.pose.position.y;
  const double twice_signed_area = ab_x * ac_y - ab_y * ac_x;
  if (!std::isfinite(twice_signed_area)) {
    return 0.0;
  }
  return 2.0 * twice_signed_area / denominator;
}

inline double normalizeAngle(double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  if (!std::isfinite(siny_cosp) || !std::isfinite(cosy_cosp)) {
    return 0.0;
  }
  return std::atan2(siny_cosp, cosy_cosp);
}

inline double headingCurvatureBetweenPoints(
    const autoware_auto_planning_msgs::msg::TrajectoryPoint &a,
    const autoware_auto_planning_msgs::msg::TrajectoryPoint &b,
    double arc_length_m) {
  if (!std::isfinite(arc_length_m) || arc_length_m <= 1.0e-6) {
    return 0.0;
  }
  const double yaw_a = yawFromQuaternion(a.pose.orientation);
  const double yaw_b = yawFromQuaternion(b.pose.orientation);
  const double yaw_delta = normalizeAngle(yaw_b - yaw_a);
  if (!std::isfinite(yaw_delta)) {
    return 0.0;
  }
  return std::abs(yaw_delta) / arc_length_m;
}

inline double adaptiveLookaheadDistance(double target_speed_mps,
                                        double current_speed_mps,
                                        double path_curvature_1pm,
                                        const LookaheadParams &params) {
  const double base_distance_m =
      speedBasedLookaheadDistance(target_speed_mps, current_speed_mps, params);
  if (!params.curvature_adaptive_enabled ||
      !std::isfinite(params.curvature_sensitivity) ||
      params.curvature_sensitivity <= 0.0 ||
      !std::isfinite(path_curvature_1pm) || path_curvature_1pm <= 0.0) {
    return base_distance_m;
  }

  const double base_min_distance_m =
      std::isfinite(params.lookahead_min_distance)
          ? std::max(0.0, params.lookahead_min_distance)
          : 0.0;
  const double configured_curvature_min_m =
      std::isfinite(params.curvature_min_distance)
          ? std::max(0.0, params.curvature_min_distance)
          : base_min_distance_m;
  const double curvature_min_distance_m =
      std::max(base_min_distance_m, configured_curvature_min_m);
  const double scale =
      1.0 / (1.0 + params.curvature_sensitivity * path_curvature_1pm);
  const double curved_distance_m = base_distance_m * scale;

  return std::min(base_distance_m,
                  std::max(curvature_min_distance_m, curved_distance_m));
}

inline double smoothLookaheadDistance(double desired_distance_m,
                                      double previous_distance_m,
                                      bool has_previous,
                                      double smoothing_alpha) {
  if (!has_previous || !std::isfinite(previous_distance_m) ||
      !std::isfinite(desired_distance_m)) {
    return desired_distance_m;
  }
  if (!std::isfinite(smoothing_alpha)) {
    return desired_distance_m;
  }

  const double alpha = std::clamp(smoothing_alpha, 0.0, 1.0);
  if (alpha >= 1.0) {
    return desired_distance_m;
  }
  return alpha * desired_distance_m + (1.0 - alpha) * previous_distance_m;
}

inline std::size_t selectForwardTrajectoryIndex(
    const autoware_auto_planning_msgs::msg::Trajectory &trajectory,
    std::size_t nearest_index, double min_forward_arc_m) {
  if (trajectory.points.empty()) {
    return 0;
  }

  const std::size_t clamped_index =
      std::min(nearest_index, trajectory.points.size() - 1);
  if (clamped_index + 1 >= trajectory.points.size()) {
    return clamped_index;
  }

  const double min_arc =
      std::isfinite(min_forward_arc_m) ? std::max(0.0, min_forward_arc_m) : 0.0;
  double accumulated_arc_m = 0.0;
  for (std::size_t i = clamped_index + 1; i < trajectory.points.size(); ++i) {
    accumulated_arc_m +=
        distance2d(trajectory.points[i - 1], trajectory.points[i]);
    if (accumulated_arc_m >= min_arc) {
      return i;
    }
  }
  return trajectory.points.size() - 1;
}

inline std::size_t selectMpcHorizonVelocityCapIndex(
    const autoware_auto_planning_msgs::msg::Trajectory &trajectory,
    std::size_t nearest_index, double min_forward_arc_m,
    bool skip_zero_index_anchor) {
  if (trajectory.points.empty()) {
    return 0;
  }
  if (skip_zero_index_anchor && nearest_index == 0) {
    return selectForwardTrajectoryIndex(trajectory, nearest_index,
                                        min_forward_arc_m);
  }
  return std::min(nearest_index, trajectory.points.size() - 1);
}

inline double estimateTrajectoryCurvature(
    const autoware_auto_planning_msgs::msg::Trajectory &trajectory,
    std::size_t start_index, double window_distance_m,
    double min_arc_length_m) {
  if (trajectory.points.size() < 3 || start_index >= trajectory.points.size()) {
    return 0.0;
  }

  const double min_arc = std::isfinite(min_arc_length_m)
                             ? std::max(1.0e-3, min_arc_length_m)
                             : 1.0e-3;
  const double max_window = std::isfinite(window_distance_m)
                                ? std::max(min_arc, window_distance_m)
                                : min_arc;
  double max_abs_curvature = 0.0;
  double accumulated_arc_m = 0.0;

  for (std::size_t i = start_index; i + 2 < trajectory.points.size(); ++i) {
    const auto &p0 = trajectory.points[i];
    const auto &p1 = trajectory.points[i + 1];
    const auto &p2 = trajectory.points[i + 2];
    const double arc_length_m = distance2d(p0, p1) + distance2d(p1, p2);
    if (!std::isfinite(arc_length_m) || arc_length_m < min_arc) {
      continue;
    }

    accumulated_arc_m += distance2d(p0, p1);
    const double curvature = curvatureFromThreePoints(p0, p1, p2);
    if (std::isfinite(curvature)) {
      max_abs_curvature = std::max(max_abs_curvature, curvature);
    }
    const double heading_curvature =
        headingCurvatureBetweenPoints(p0, p2, arc_length_m);
    if (std::isfinite(heading_curvature)) {
      max_abs_curvature = std::max(max_abs_curvature, heading_curvature);
    }
    if (accumulated_arc_m >= max_window) {
      break;
    }
  }

  return max_abs_curvature;
}

inline double estimateSignedTrajectoryCurvature(
    const autoware_auto_planning_msgs::msg::Trajectory &trajectory,
    std::size_t start_index, double window_distance_m,
    double min_arc_length_m) {
  if (trajectory.points.size() < 3 || start_index >= trajectory.points.size()) {
    return 0.0;
  }

  const double min_arc = std::isfinite(min_arc_length_m)
                             ? std::max(1.0e-3, min_arc_length_m)
                             : 1.0e-3;
  const double max_window = std::isfinite(window_distance_m)
                                ? std::max(min_arc, window_distance_m)
                                : min_arc;
  double selected_curvature = 0.0;
  double accumulated_arc_m = 0.0;

  for (std::size_t i = start_index; i + 2 < trajectory.points.size(); ++i) {
    const auto &p0 = trajectory.points[i];
    const auto &p1 = trajectory.points[i + 1];
    const auto &p2 = trajectory.points[i + 2];
    const double arc_length_m = distance2d(p0, p1) + distance2d(p1, p2);
    if (!std::isfinite(arc_length_m) || arc_length_m < min_arc) {
      continue;
    }

    accumulated_arc_m += distance2d(p0, p1);
    const double geometry_curvature =
        signedCurvatureFromThreePoints(p0, p1, p2);
    if (std::isfinite(geometry_curvature) &&
        std::abs(geometry_curvature) > std::abs(selected_curvature)) {
      selected_curvature = geometry_curvature;
    }

    const double yaw_a = yawFromQuaternion(p0.pose.orientation);
    const double yaw_b = yawFromQuaternion(p2.pose.orientation);
    const double heading_curvature =
        normalizeAngle(yaw_b - yaw_a) / arc_length_m;
    if (std::isfinite(heading_curvature) &&
        std::abs(heading_curvature) > std::abs(selected_curvature)) {
      selected_curvature = heading_curvature;
    }
    if (accumulated_arc_m >= max_window) {
      break;
    }
  }

  return selected_curvature;
}

} // namespace simple_pure_pursuit

#endif // SIMPLE_PURE_PURSUIT_LOOKAHEAD_HPP_
