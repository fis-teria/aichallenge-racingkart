#include "simple_trajectory_generator/execution_profile.hpp"

#include <builtin_interfaces/msg/duration.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace simple_trajectory_generator {
namespace {

using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

constexpr double kCanonicalSpeedCeilingMps = 10.0;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kMinimumSegmentLengthM = 1.0e-6;
constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

bool finitePoint(const TrajectoryPoint &point) {
  const auto &p = point.pose.position;
  const auto &q = point.pose.orientation;
  const double quaternion_norm =
      std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) &&
         std::abs(quaternion_norm - 1.0) <= kQuaternionNormTolerance &&
         std::isfinite(point.longitudinal_velocity_mps) &&
         point.longitudinal_velocity_mps > 0.0F;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double shortestYawDifference(const double from_rad, const double to_rad) {
  return std::remainder(to_rad - from_rad, kTwoPi);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw_rad) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw_rad);
  q.w = std::cos(0.5 * yaw_rad);
  return q;
}

builtin_interfaces::msg::Duration
durationFromNanoseconds(const std::int64_t nanoseconds) {
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<std::int32_t>(nanoseconds / kNanosecondsPerSecond);
  duration.nanosec =
      static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
  return duration;
}

} // namespace

ExecutionProfileResult buildExecutionProfile(
    const autoware_auto_planning_msgs::msg::Trajectory &source,
    const ExecutionProfileConfig &config) {
  ExecutionProfileResult result;
  if (!std::isfinite(config.max_speed_mps) || config.max_speed_mps <= 0.0 ||
      config.max_speed_mps > kCanonicalSpeedCeilingMps) {
    result.reason = "invalid_max_speed_mps";
    return result;
  }
  if (!std::isfinite(config.max_arc_spacing_m) ||
      config.max_arc_spacing_m <= 0.0 || config.max_arc_spacing_m > 0.25) {
    result.reason = "invalid_max_arc_spacing_m";
    return result;
  }
  if (!std::isfinite(config.max_yaw_step_rad) ||
      config.max_yaw_step_rad <= 0.0 || config.max_yaw_step_rad > 0.05 ||
      config.max_output_points < 2U) {
    result.reason = "invalid_geometry_config";
    return result;
  }
  if (source.points.size() < 2U) {
    result.reason = "too_few_source_points";
    return result;
  }

  double execution_speed_mps = config.max_speed_mps;
  for (const auto &point : source.points) {
    if (!finitePoint(point)) {
      result.reason = "invalid_source_point";
      return result;
    }
    execution_speed_mps =
        std::min(execution_speed_mps,
                 static_cast<double>(point.longitudinal_velocity_mps));
  }
  if (!std::isfinite(execution_speed_mps) || execution_speed_mps <= 0.0) {
    result.reason = "invalid_execution_speed";
    return result;
  }

  auto &output = result.trajectory;
  output.header = source.header;
  output.points.reserve(
      std::min(config.max_output_points, source.points.size() * 6U));

  TrajectoryPoint first = source.points.front();
  first.longitudinal_velocity_mps = static_cast<float>(execution_speed_mps);
  first.lateral_velocity_mps = 0.0F;
  first.acceleration_mps2 = 0.0F;
  first.heading_rate_rps = 0.0F;
  first.time_from_start = durationFromNanoseconds(0);
  output.points.push_back(first);

  std::int64_t accumulated_nanoseconds = 0;
  for (std::size_t source_index = 1U; source_index < source.points.size();
       ++source_index) {
    const auto &previous = source.points[source_index - 1U];
    const auto &current = source.points[source_index];
    const double dx = current.pose.position.x - previous.pose.position.x;
    const double dy = current.pose.position.y - previous.pose.position.y;
    const double dz = current.pose.position.z - previous.pose.position.z;
    const double distance_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(distance_m) || distance_m <= kMinimumSegmentLengthM) {
      result.reason = "degenerate_source_segment";
      result.trajectory.points.clear();
      return result;
    }

    const double previous_yaw_rad =
        yawFromQuaternion(previous.pose.orientation);
    const double yaw_difference_rad = shortestYawDifference(
        previous_yaw_rad, yawFromQuaternion(current.pose.orientation));
    const auto arc_steps = static_cast<std::size_t>(
        std::ceil(distance_m / config.max_arc_spacing_m));
    const auto yaw_steps = static_cast<std::size_t>(
        std::ceil(std::abs(yaw_difference_rad) / config.max_yaw_step_rad));
    const std::size_t steps = std::max<std::size_t>({1U, arc_steps, yaw_steps});
    if (steps > config.max_output_points ||
        output.points.size() > config.max_output_points - steps) {
      result.reason = "output_point_limit_exceeded";
      result.trajectory.points.clear();
      return result;
    }

    const double step_distance_m = distance_m / static_cast<double>(steps);
    const double step_duration_sec = step_distance_m / execution_speed_mps;
    if (!std::isfinite(step_duration_sec) || step_duration_sec <= 0.0) {
      result.reason = "invalid_segment_duration";
      result.trajectory.points.clear();
      return result;
    }
    const double step_nanoseconds_double =
        std::ceil(step_duration_sec * kNanosecondsPerSecond);
    if (!std::isfinite(step_nanoseconds_double) ||
        step_nanoseconds_double < 1.0 ||
        step_nanoseconds_double >
            static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
      result.reason = "segment_duration_overflow";
      result.trajectory.points.clear();
      return result;
    }
    const auto step_nanoseconds =
        static_cast<std::int64_t>(step_nanoseconds_double);

    for (std::size_t step = 1U; step <= steps; ++step) {
      if (accumulated_nanoseconds >
          std::numeric_limits<std::int32_t>::max() * kNanosecondsPerSecond -
              step_nanoseconds) {
        result.reason = "trajectory_duration_overflow";
        result.trajectory.points.clear();
        return result;
      }
      accumulated_nanoseconds += step_nanoseconds;
      const double ratio =
          static_cast<double>(step) / static_cast<double>(steps);
      TrajectoryPoint point;
      point.pose.position.x = previous.pose.position.x + ratio * dx;
      point.pose.position.y = previous.pose.position.y + ratio * dy;
      point.pose.position.z = previous.pose.position.z + ratio * dz;
      point.pose.orientation =
          quaternionFromYaw(previous_yaw_rad + ratio * yaw_difference_rad);
      point.longitudinal_velocity_mps = static_cast<float>(execution_speed_mps);
      point.lateral_velocity_mps = 0.0F;
      point.acceleration_mps2 = 0.0F;
      point.heading_rate_rps = 0.0F;
      point.time_from_start = durationFromNanoseconds(accumulated_nanoseconds);
      output.points.push_back(point);
    }
  }

  result.valid = true;
  result.execution_speed_mps = execution_speed_mps;
  return result;
}

} // namespace simple_trajectory_generator
