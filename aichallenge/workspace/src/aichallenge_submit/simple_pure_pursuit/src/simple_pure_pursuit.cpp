#include "simple_pure_pursuit/simple_pure_pursuit.hpp"
#include "simple_pure_pursuit/pass_warmup_acquisition.hpp"

#include "overtake_transport_contract/c002ay0_canonical.hpp"
#include "simple_pure_pursuit/overtake_override_contract.hpp"

#include "simple_pure_pursuit/delay_compensation.hpp"

#ifdef SIMPLE_PURE_PURSUIT_TIMING_DIAGNOSTIC
#include "pp_callback_span_recorder.hpp"
#define PP_DIAGNOSTIC_CONCAT_INNER(lhs, rhs) lhs##rhs
#define PP_DIAGNOSTIC_CONCAT(lhs, rhs) PP_DIAGNOSTIC_CONCAT_INNER(lhs, rhs)
#define PP_DIAGNOSTIC_SCOPE(kind)                                              \
  simple_pure_pursuit::timing_diagnostic::CallbackSpanScope                    \
  PP_DIAGNOSTIC_CONCAT(pp_diagnostic_scope_, __LINE__)(kind)
#else
#define PP_DIAGNOSTIC_SCOPE(kind)
#endif

#include <ament_index_cpp/get_package_prefix.hpp>
#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

namespace simple_pure_pursuit {

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

namespace {
constexpr std::size_t kMaxMpcHorizonNearestIndex = 3;
constexpr double kMpcHorizonVelocityCapForwardArcM = 0.25;

bool validPositiveDecimalId(const std::string &value) {
  return !value.empty() && value != "0" &&
         std::all_of(value.begin(), value.end(), [](unsigned char character) {
           return std::isdigit(character) != 0;
         });
}

std::uint64_t steadyNowNanoseconds() noexcept {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

std::string ay0ShadowWorkerPath() {
  try {
    return ament_index_cpp::get_package_prefix("overtake_transport_contract") +
           "/lib/overtake_transport_contract/c002ay0_shadow_worker";
  } catch (...) {
    return {};
  }
}

overtake_transport_contract::c002ay0::FixedTime
toAy0FixedTime(const builtin_interfaces::msg::Time &source) noexcept {
  return {source.sec, source.nanosec};
}

overtake_transport_contract::c002ay0::FixedTime
addSeconds(const builtin_interfaces::msg::Time &source,
           double seconds) noexcept {
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return toAy0FixedTime(source);
  }
  const auto additional_ns =
      static_cast<std::int64_t>(seconds * kNanosecondsPerSecond);
  const std::int64_t source_ns =
      static_cast<std::int64_t>(source.sec) * kNanosecondsPerSecond +
      static_cast<std::int64_t>(source.nanosec);
  const std::int64_t result_ns = source_ns + additional_ns;
  return {static_cast<std::int32_t>(result_ns / kNanosecondsPerSecond),
          static_cast<std::uint32_t>(result_ns % kNanosecondsPerSecond)};
}

std::string siblingShadowSupervisorPath() {
  std::array<char, 4096U> path{};
  const ssize_t length =
      readlink("/proc/self/exe", path.data(), path.size() - 1U);
  if (length <= 0 || static_cast<std::size_t>(length) >= path.size()) {
    return {};
  }
  path[static_cast<std::size_t>(length)] = '\0';
  std::string executable(path.data());
  const auto separator = executable.find_last_of('/');
  if (separator == std::string::npos) {
    return {};
  }
  executable.resize(separator + 1U);
  executable += "controller_applied_shadow_supervisor";
  return executable;
}

aw2_shadow::FixedTime
toFixedTime(const builtin_interfaces::msg::Time &source) noexcept {
  return {source.sec, source.nanosec};
}

aw2_shadow::FixedTime
toFixedTime(const builtin_interfaces::msg::Duration &source) noexcept {
  return {source.sec, source.nanosec};
}

aw2_shadow::FixedCommand
toFixedCommand(const AckermannControlCommand &source) noexcept {
  aw2_shadow::FixedCommand destination;
  destination.command_stamp = toFixedTime(source.stamp);
  destination.lateral_stamp = toFixedTime(source.lateral.stamp);
  destination.steering_tire_angle_rad = source.lateral.steering_tire_angle;
  destination.steering_tire_rotation_rate_radps =
      source.lateral.steering_tire_rotation_rate;
  destination.longitudinal_stamp = toFixedTime(source.longitudinal.stamp);
  destination.longitudinal_speed_mps = source.longitudinal.speed;
  destination.longitudinal_acceleration_mps2 = source.longitudinal.acceleration;
  destination.longitudinal_jerk_mps3 = source.longitudinal.jerk;
  return destination;
}

aw2_shadow::FixedTrajectoryPoint
toFixedPoint(const TrajectoryPoint &source) noexcept {
  aw2_shadow::FixedTrajectoryPoint destination;
  destination.time_from_start = toFixedTime(source.time_from_start);
  destination.position_x_m = source.pose.position.x;
  destination.position_y_m = source.pose.position.y;
  destination.position_z_m = source.pose.position.z;
  destination.orientation_x = source.pose.orientation.x;
  destination.orientation_y = source.pose.orientation.y;
  destination.orientation_z = source.pose.orientation.z;
  destination.orientation_w = source.pose.orientation.w;
  destination.longitudinal_velocity_mps = source.longitudinal_velocity_mps;
  destination.lateral_velocity_mps = source.lateral_velocity_mps;
  destination.acceleration_mps2 = source.acceleration_mps2;
  destination.heading_rate_rps = source.heading_rate_rps;
  destination.front_wheel_angle_rad = source.front_wheel_angle_rad;
  destination.rear_wheel_angle_rad = source.rear_wheel_angle_rad;
  return destination;
}

bool copyFixedGeometryInterval(
    const Trajectory &source, std::size_t first_source_index,
    std::size_t last_source_index, std::size_t speed_cap_source_index,
    std::size_t curvature_last_read_source_index,
    std::size_t lookahead_selected_source_index,
    std::size_t required_horizon_end_source_index,
    bool lookahead_endpoint_fallback,
    aw2_shadow::FixedGeometry &destination) noexcept {
  if (source.points.empty() || first_source_index >= source.points.size() ||
      last_source_index < first_source_index ||
      last_source_index >= source.points.size() ||
      last_source_index - first_source_index + 1U >
          aw2_shadow::kMaxGeometryPoints ||
      !aw2_shadow::copyFixedString(source.header.frame_id,
                                   destination.frame_id)) {
    destination = {};
    return false;
  }
  destination.source_stamp = toFixedTime(source.header.stamp);
  destination.original_point_count =
      static_cast<std::uint32_t>(source.points.size());
  destination.first_source_index =
      static_cast<std::uint32_t>(first_source_index);
  destination.last_source_index = static_cast<std::uint32_t>(last_source_index);
  destination.nearest_source_index =
      static_cast<std::uint32_t>(first_source_index);
  destination.speed_cap_source_index =
      static_cast<std::uint32_t>(speed_cap_source_index);
  destination.curvature_last_read_source_index =
      static_cast<std::uint32_t>(curvature_last_read_source_index);
  destination.lookahead_selected_source_index =
      static_cast<std::uint32_t>(lookahead_selected_source_index);
  destination.required_horizon_end_source_index =
      static_cast<std::uint32_t>(required_horizon_end_source_index);
  destination.lookahead_endpoint_fallback = lookahead_endpoint_fallback;
  destination.point_count =
      static_cast<std::uint32_t>(last_source_index - first_source_index + 1U);
  for (std::size_t index = 0U;
       index < static_cast<std::size_t>(destination.point_count); ++index) {
    destination.points[index] =
        toFixedPoint(source.points[first_source_index + index]);
  }
  return true;
}

std::optional<std::size_t>
requiredHorizonEndIndex(const Trajectory &trajectory, std::size_t nearest_index,
                        double required_horizon_m) noexcept {
  if (nearest_index >= trajectory.points.size() ||
      !std::isfinite(required_horizon_m) || required_horizon_m <= 0.0) {
    return std::nullopt;
  }
  double arc_m = 0.0;
  for (std::size_t index = nearest_index + 1U; index < trajectory.points.size();
       ++index) {
    const auto &previous = trajectory.points[index - 1U].pose.position;
    const auto &current = trajectory.points[index].pose.position;
    const double segment_m =
        std::hypot(current.x - previous.x, current.y - previous.y);
    if (!std::isfinite(segment_m) || segment_m < 0.0) {
      return std::nullopt;
    }
    arc_m += segment_m;
    if (!std::isfinite(arc_m)) {
      return std::nullopt;
    }
    if (arc_m >= required_horizon_m) {
      return index;
    }
  }
  return std::nullopt;
}

bool odometryControlValuesValid(const Odometry &odometry) {
  const auto &position = odometry.pose.pose.position;
  const auto &orientation = odometry.pose.pose.orientation;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
         std::isfinite(orientation.w) &&
         std::isfinite(odometry.twist.twist.linear.x);
}

TrajectoryPoint interpolateTrajectoryPoint(const TrajectoryPoint &from,
                                           const TrajectoryPoint &to,
                                           double ratio) {
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  TrajectoryPoint point = from;
  point.pose.position.x =
      from.pose.position.x +
      (to.pose.position.x - from.pose.position.x) * clamped_ratio;
  point.pose.position.y =
      from.pose.position.y +
      (to.pose.position.y - from.pose.position.y) * clamped_ratio;
  point.pose.position.z =
      from.pose.position.z +
      (to.pose.position.z - from.pose.position.z) * clamped_ratio;
  const double from_yaw = tf2::getYaw(from.pose.orientation);
  const double to_yaw = tf2::getYaw(to.pose.orientation);
  const double yaw = from_yaw + std::atan2(std::sin(to_yaw - from_yaw),
                                           std::cos(to_yaw - from_yaw)) *
                                    clamped_ratio;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  point.pose.orientation.x = orientation.x();
  point.pose.orientation.y = orientation.y();
  point.pose.orientation.z = orientation.z();
  point.pose.orientation.w = orientation.w();
  point.longitudinal_velocity_mps =
      static_cast<float>(static_cast<double>(from.longitudinal_velocity_mps) +
                         (static_cast<double>(to.longitudinal_velocity_mps) -
                          static_cast<double>(from.longitudinal_velocity_mps)) *
                             clamped_ratio);
  return point;
}

// 検証済み空間profileの終端が基準trajectory点の間にある場合、その位置へ
// 補間点を追加する。終端より先は使わず、PPが最低2点を追えるようにする。
std::optional<std::size_t>
insertSpatialHorizonEndpoint(Trajectory &trajectory, std::size_t nearest_index,
                             double endpoint_arc_m) {
  if (nearest_index >= trajectory.points.size() ||
      !std::isfinite(endpoint_arc_m) || endpoint_arc_m <= 1.0e-6) {
    return std::nullopt;
  }
  double accumulated_arc_m = 0.0;
  for (std::size_t i = nearest_index + 1U; i < trajectory.points.size(); ++i) {
    const auto &previous = trajectory.points[i - 1U].pose.position;
    const auto &current = trajectory.points[i].pose.position;
    const double segment_m =
        std::hypot(current.x - previous.x, current.y - previous.y);
    if (!std::isfinite(segment_m) || segment_m <= 1.0e-9) {
      continue;
    }
    const double next_arc_m = accumulated_arc_m + segment_m;
    if (next_arc_m + kSpatialProfileEndpointToleranceM < endpoint_arc_m) {
      accumulated_arc_m = next_arc_m;
      continue;
    }
    if (std::abs(next_arc_m - endpoint_arc_m) <=
        kSpatialProfileEndpointToleranceM) {
      return i;
    }
    const double ratio = (endpoint_arc_m - accumulated_arc_m) / segment_m;
    const auto endpoint_point = interpolateTrajectoryPoint(
        trajectory.points[i - 1U], trajectory.points[i], ratio);
    const double reconstructed_endpoint_arc_m =
        accumulated_arc_m +
        std::hypot(endpoint_point.pose.position.x - previous.x,
                   endpoint_point.pose.position.y - previous.y);
    if (!std::isfinite(reconstructed_endpoint_arc_m) ||
        std::abs(reconstructed_endpoint_arc_m - endpoint_arc_m) >
            kSpatialProfileEndpointToleranceM) {
      return std::nullopt;
    }
    trajectory.points.insert(trajectory.points.begin() +
                                 static_cast<std::ptrdiff_t>(i),
                             endpoint_point);
    return i;
  }
  return std::nullopt;
}

double trajectoryArcLength(const Trajectory &trajectory) {
  if (trajectory.points.size() < 2) {
    return 0.0;
  }

  double length_m = 0.0;
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto &prev = trajectory.points[i - 1].pose.position;
    const auto &curr = trajectory.points[i].pose.position;
    length_m += std::hypot(curr.x - prev.x, curr.y - prev.y);
  }
  return length_m;
}

bool trajectoryValuesValid(const Trajectory &trajectory) {
  for (const auto &point : trajectory.points) {
    const auto &pos = point.pose.position;
    const auto &orientation = point.pose.orientation;
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) ||
        !std::isfinite(pos.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w) ||
        !std::isfinite(point.longitudinal_velocity_mps) ||
        point.longitudinal_velocity_mps < 0.0) {
      return false;
    }
  }
  return true;
}

double firstPointDistanceToEgo(const Trajectory &trajectory,
                               const Odometry &odometry) {
  if (trajectory.points.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto &first = trajectory.points.front().pose.position;
  const auto &ego = odometry.pose.pose.position;
  return std::hypot(first.x - ego.x, first.y - ego.y);
}

std::optional<std::string> jsonStringField(const std::string &json,
                                           const std::string &key) {
  const std::string key_token = "\"" + key + "\"";
  const auto key_pos = json.find(key_token);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = json.find(':', key_pos + key_token.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto value_start = json.find('"', colon_pos + 1);
  if (value_start == std::string::npos) {
    return std::nullopt;
  }
  const auto value_end = json.find('"', value_start + 1);
  if (value_end == std::string::npos || value_end <= value_start) {
    return std::nullopt;
  }
  return json.substr(value_start + 1, value_end - value_start - 1);
}

std::optional<double> jsonNumberField(const std::string &json,
                                      const std::string &key) {
  const std::string key_token = "\"" + key + "\"";
  const auto key_pos = json.find(key_token);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = json.find(':', key_pos + key_token.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }

  std::size_t value_start = colon_pos + 1;
  while (value_start < json.size() &&
         std::isspace(static_cast<unsigned char>(json[value_start]))) {
    ++value_start;
  }
  std::size_t value_end = value_start;
  while (value_end < json.size()) {
    const char c = json[value_end];
    if (!(std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+' ||
          c == '.' || c == 'e' || c == 'E')) {
      break;
    }
    ++value_end;
  }
  if (value_end == value_start) {
    return std::nullopt;
  }

  try {
    return std::stod(json.substr(value_start, value_end - value_start));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<std::int64_t> jsonIntegerField(const std::string &json,
                                             const std::string &key) {
  const std::string key_token = "\"" + key + "\"";
  const auto key_pos = json.find(key_token);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = json.find(':', key_pos + key_token.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }
  std::size_t value_start = colon_pos + 1;
  while (value_start < json.size() &&
         std::isspace(static_cast<unsigned char>(json[value_start]))) {
    ++value_start;
  }
  std::size_t value_end = value_start;
  if (value_end < json.size() && json[value_end] == '-') {
    ++value_end;
  }
  while (value_end < json.size() &&
         std::isdigit(static_cast<unsigned char>(json[value_end]))) {
    ++value_end;
  }
  if (value_end == value_start ||
      (json[value_start] == '-' && value_end == value_start + 1)) {
    return std::nullopt;
  }
  if (value_end < json.size() &&
      (json[value_end] == '.' || json[value_end] == 'e' ||
       json[value_end] == 'E')) {
    return std::nullopt;
  }
  try {
    return std::stoll(json.substr(value_start, value_end - value_start));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<bool> jsonBooleanField(const std::string &json,
                                     const std::string &key) {
  const std::string key_token = "\"" + key + "\"";
  const auto key_pos = json.find(key_token);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = json.find(':', key_pos + key_token.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }
  std::size_t value_start = colon_pos + 1;
  while (value_start < json.size() &&
         std::isspace(static_cast<unsigned char>(json[value_start]))) {
    ++value_start;
  }
  if (json.compare(value_start, 4, "true") == 0) {
    return true;
  }
  if (json.compare(value_start, 5, "false") == 0) {
    return false;
  }
  return std::nullopt;
}

std::uint64_t makeProducerInstanceId() {
  static std::atomic<std::uint64_t> instance_counter{1U};
  const auto steady_ticks =
      std::chrono::steady_clock::now().time_since_epoch().count();
  std::uint64_t instance_id =
      static_cast<std::uint64_t>(steady_ticks) ^
      instance_counter.fetch_add(1U, std::memory_order_relaxed);
  if (instance_id == 0U) {
    instance_id = instance_counter.fetch_add(1U, std::memory_order_relaxed);
  }
  return instance_id == 0U ? 1U : instance_id;
}

} // namespace

bool stateLatticeV2IdentityMatches(
    const multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity &lhs,
    const multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity &rhs) {
  return lhs.producer_instance_id == rhs.producer_instance_id &&
         lhs.session_id == rhs.session_id &&
         lhs.proposal_sequence == rhs.proposal_sequence &&
         lhs.plan_generation == rhs.plan_generation &&
         lhs.source_generation == rhs.source_generation &&
         lhs.source_stamp.sec == rhs.source_stamp.sec &&
         lhs.source_stamp.nanosec == rhs.source_stamp.nanosec &&
         lhs.frame_id == rhs.frame_id &&
         lhs.canonical_sha256 == rhs.canonical_sha256;
}

bool stateLatticeV2BaseAttestationSemanticMatch(
    const StateLatticeV2BaseAttestation &previous,
    const StateLatticeV2BaseAttestation &candidate) {
  const auto &lhs = previous.base;
  const auto &rhs = candidate.base;
  const auto same_time = [](const auto &left, const auto &right) {
    return left.sec == right.sec && left.nanosec == right.nanosec;
  };
  return previous.schema_version == candidate.schema_version &&
         previous.producer_instance_id == candidate.producer_instance_id &&
         previous.session_id == candidate.session_id &&
         previous.header.frame_id == candidate.header.frame_id &&
         lhs.schema_version == rhs.schema_version &&
         lhs.authority_eligible == rhs.authority_eligible &&
         lhs.frame_id == rhs.frame_id &&
         lhs.race_arm_epoch == rhs.race_arm_epoch &&
         lhs.controller_instance_id == rhs.controller_instance_id &&
         same_time(lhs.lease_valid_until, rhs.lease_valid_until) &&
         lhs.base_source_kind == rhs.base_source_kind &&
         same_time(lhs.base_source_stamp, rhs.base_source_stamp) &&
         lhs.base_source_generation == rhs.base_source_generation &&
         lhs.base_original_point_count == rhs.base_original_point_count &&
         lhs.first_source_index == rhs.first_source_index &&
         lhs.last_source_index == rhs.last_source_index &&
         lhs.nearest_source_index == rhs.nearest_source_index &&
         lhs.base_source_digest_state == rhs.base_source_digest_state &&
         lhs.canonical_algorithm_version == rhs.canonical_algorithm_version &&
         lhs.base_geometry_sha256 == rhs.base_geometry_sha256 &&
         lhs.base_source_sha256 == rhs.base_source_sha256 &&
         lhs.controller_implementation_sha256 ==
             rhs.controller_implementation_sha256 &&
         lhs.controller_config_sha256 == rhs.controller_config_sha256;
}

bool stateLatticeV2BaseAttestationReusable(
    const StateLatticeV2BaseAttestation &previous,
    const StateLatticeV2BaseAttestation &candidate,
    const builtin_interfaces::msg::Time &now_ros) {
  const auto time_ns =
      [](const builtin_interfaces::msg::Time &value) -> std::int64_t {
    if (value.sec < 0 || value.nanosec >= 1000000000U) {
      return std::int64_t{-1};
    }
    return static_cast<std::int64_t>(value.sec) *
               static_cast<std::int64_t>(1000000000LL) +
           static_cast<std::int64_t>(value.nanosec);
  };
  const auto now_ns = time_ns(now_ros);
  const auto previous_expiry_ns = time_ns(previous.base.lease_valid_until);
  return now_ns > 0 && previous_expiry_ns > 0 &&
         now_ns < previous_expiry_ns &&
         stateLatticeV2BaseAttestationSemanticMatch(previous, candidate);
}

bool stateLatticeV2CurrentAvailabilityRejected(
    const overtake_transport_contract::state_lattice_v2::CycleResult &result) {
  if (result.run_invalid) {
    return true;
  }
  if (!result.availability_identity.has_value()) {
    return false;
  }
  return std::any_of(
      result.events.begin(), result.events.begin() + result.event_count,
      [&result](const auto &event) {
        return event.reject_reason !=
                   overtake_transport_contract::state_lattice_v2::RejectReason::
                       kNone &&
               event.identity.has_value() &&
               stateLatticeV2IdentityMatches(
                   event.identity.value(),
                   result.availability_identity.value());
      });
}

std::optional<Trajectory> stateLatticeV2ProposalToTrajectory(
    const AuthorizedCartesianTrajectoryV2 &proposal) {
  using overtake_transport_contract::c002ay0::ValidationError;
  using overtake_transport_contract::c002ay0::validateAuthorizedTrajectoryV1;

  const auto &source = proposal.proposal;
  if (proposal.schema_version !=
          AuthorizedCartesianTrajectoryV2::SCHEMA_V2_NON_AUTHORITATIVE ||
      proposal.header.frame_id != source.frame_id ||
      proposal.identity.frame_id != source.frame_id ||
      proposal.header.stamp.sec != source.plan_stamp.sec ||
      proposal.header.stamp.nanosec != source.plan_stamp.nanosec ||
      proposal.identity.canonical_sha256 != source.payload_sha256 ||
      validateAuthorizedTrajectoryV1(source) != ValidationError::NONE) {
    return std::nullopt;
  }

  Trajectory result;
  result.header = proposal.header;
  result.points.reserve(source.points.size());
  builtin_interfaces::msg::Duration previous_time{};
  bool has_previous_time = false;
  for (const auto &point : source.points) {
    const bool monotonic_time =
        !has_previous_time || point.time_from_start.sec > previous_time.sec ||
        (point.time_from_start.sec == previous_time.sec &&
         point.time_from_start.nanosec > previous_time.nanosec);
    if (!monotonic_time ||
        point.longitudinal_velocity_mps > 10.0F) {
      return std::nullopt;
    }
    TrajectoryPoint converted;
    converted.pose.position.x = point.position_x_m;
    converted.pose.position.y = point.position_y_m;
    converted.pose.position.z = point.position_z_m;
    converted.pose.orientation.x = point.orientation_x;
    converted.pose.orientation.y = point.orientation_y;
    converted.pose.orientation.z = point.orientation_z;
    converted.pose.orientation.w = point.orientation_w;
    converted.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    converted.lateral_velocity_mps = point.lateral_velocity_mps;
    converted.acceleration_mps2 = point.acceleration_mps2;
    converted.heading_rate_rps = point.heading_rate_rps;
    converted.front_wheel_angle_rad = point.front_wheel_angle_rad;
    converted.rear_wheel_angle_rad = point.rear_wheel_angle_rad;
    converted.time_from_start = point.time_from_start;
    result.points.push_back(std::move(converted));
    previous_time = point.time_from_start;
    has_previous_time = true;
  }
  return result;
}

SimplePurePursuit::SimplePurePursuit(const rclcpp::NodeOptions &options)
    : Node("simple_pure_pursuit", options),
      // initialize parameters
      wheel_base_(declare_parameter<float>("wheel_base", 1.087)),
      lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
      lookahead_min_distance_(
          declare_parameter<float>("lookahead_min_distance", 1.0)),
      speed_proportional_gain_(
          declare_parameter<float>("speed_proportional_gain", 1.0)),
      use_external_target_vel_(
          declare_parameter<bool>("use_external_target_vel", false)),
      external_target_vel_(
          declare_parameter<float>("external_target_vel", 0.0)),
      steering_tire_angle_gain_(
          declare_parameter<float>("steering_tire_angle_gain", 1.0)),
      debug_publish_period_sec_(
          declare_parameter<float>("debug_publish_period_sec", 0.25)),
      max_odom_age_sec_(declare_parameter<float>("max_odom_age_sec", 0.20)),
      max_trajectory_age_sec_(
          declare_parameter<float>("max_trajectory_age_sec", 0.50)),
      max_override_age_sec_(
          declare_parameter<float>("max_override_age_sec", 0.50)),
      stop_on_stale_input_(
          declare_parameter<bool>("stop_on_stale_input", true)),
      diagnostic_throttle_sec_(
          declare_parameter<float>("diagnostic_throttle_sec", 1.0)),
      use_mpc_predicted_horizon_(
          declare_parameter<bool>("use_mpc_predicted_horizon", false)),
      max_mpc_horizon_age_sec_(
          declare_parameter<float>("max_mpc_horizon_age_sec", 0.15)),
      min_mpc_horizon_points_(
          declare_parameter<int>("min_mpc_horizon_points", 5)),
      max_mpc_horizon_start_distance_m_(
          declare_parameter<float>("max_mpc_horizon_start_distance_m", 2.0)),
      min_mpc_horizon_arc_length_m_(
          declare_parameter<float>("min_mpc_horizon_arc_length_m", 2.0)),
      require_solved_mpc_health_for_horizon_(declare_parameter<bool>(
          "require_solved_mpc_health_for_horizon", false)),
      max_mpc_health_age_sec_(
          declare_parameter<float>("max_mpc_health_age_sec", 0.30)),
      require_matching_overtake_horizon_contract_(declare_parameter<bool>(
          "require_matching_overtake_horizon_contract", false)),
      use_overtake_reference_override_(
          declare_parameter<bool>("use_overtake_reference_override", false)),
      require_overtake_reference_override_fresh_(declare_parameter<bool>(
          "require_overtake_reference_override_fresh", false)),
      state_lattice_v4_poc_identity_gate_enabled_(declare_parameter<bool>(
          "state_lattice_v4_poc_identity_gate_enabled", false)),
      state_lattice_v4_poc_command_activation_enabled_(declare_parameter<bool>(
          "state_lattice_v4_poc_command_activation_enabled", false)),
      recovery_mode_(declare_parameter<bool>("recovery_mode", false)),
      recovery_status_timeout_sec_(
          declare_parameter<float>("recovery_status_timeout_sec", 0.20)),
      overtake_override_timeout_sec_(
          declare_parameter<float>("overtake_override_timeout_sec", 0.50)),
      overtake_short_spatial_horizon_v_max_mps_(declare_parameter<float>(
          "overtake_short_spatial_horizon_v_max_mps", 0.20)),
      overtake_spatial_horizon_min_arc_m_(
          declare_parameter<float>("overtake_spatial_horizon_min_arc_m", 0.50)),
      overtake_spatial_horizon_min_time_sec_(declare_parameter<float>(
          "overtake_spatial_horizon_min_time_sec", 0.75)),
      overtake_spatial_horizon_response_delay_sec_(declare_parameter<float>(
          "overtake_spatial_horizon_response_delay_sec", 0.25)),
      overtake_spatial_horizon_brake_decel_mps2_(declare_parameter<float>(
          "overtake_spatial_horizon_brake_decel_mps2", 1.0)),
      curvature_adaptive_lookahead_enabled_(declare_parameter<bool>(
          "curvature_adaptive_lookahead_enabled", true)),
      curvature_lookahead_min_distance_(
          declare_parameter<float>("curvature_lookahead_min_distance", 3.5)),
      curvature_lookahead_sensitivity_(
          declare_parameter<float>("curvature_lookahead_sensitivity", 8.0)),
      curvature_lookahead_window_ratio_(
          declare_parameter<float>("curvature_lookahead_window_ratio", 2.0)),
      curvature_lookahead_max_window_distance_(declare_parameter<float>(
          "curvature_lookahead_max_window_distance", 10.0)),
      curvature_lookahead_min_arc_length_(
          declare_parameter<float>("curvature_lookahead_min_arc_length", 1.0)),
      curvature_lookahead_smoothing_alpha_(declare_parameter<float>(
          "curvature_lookahead_smoothing_alpha", 0.35)),
      pp_control_delay_sec_(
          declare_parameter<float>("pp_control_delay_sec", 0.0)),
      pp_prediction_dt_sec_(
          declare_parameter<float>("pp_prediction_dt_sec", 0.02)),
      steering_time_constant_sec_(
          declare_parameter<float>("steering_time_constant_sec", 0.30)),
      steering_status_timeout_sec_(
          declare_parameter<float>("steering_status_timeout_sec", 0.20)),
      min_velocity_for_delay_compensation_mps_(declare_parameter<float>(
          "min_velocity_for_delay_compensation_mps", 0.20)),
      horizon_curvature_feedforward_gain_(
          declare_parameter<float>("horizon_curvature_feedforward_gain", 0.0)),
      horizon_curvature_feedforward_max_rad_(declare_parameter<float>(
          "horizon_curvature_feedforward_max_rad", 0.08)),
      free_run_live_exact_ack_enabled_(
          declare_parameter<bool>("free_run_live_exact_ack_enabled", false)),
      pp_core_exact_snapshot_enabled_(
          declare_parameter<bool>("pp_core_exact_snapshot_enabled", false)),
      free_run_live_exact_hard_steering_limit_rad_(declare_parameter<float>(
          "free_run_live_exact_hard_steering_limit_rad", 0.64)),
      free_run_live_exact_hard_steering_rate_limit_radps_(
          declare_parameter<float>(
              "free_run_live_exact_hard_steering_rate_limit_radps", 0.5)),
      steering_command_nominal_dt_sec_(
          declare_parameter<float>("steering_command_nominal_dt_sec", 0.01)),
      producer_instance_id_(makeProducerInstanceId()) {
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ =
      create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_recovery_cmd_ = create_publisher<RecoveryControlCommand>(
      "output/recovery_control_cmd", 1);
  pub_lookahead_point_ =
      create_publisher<PointStamped>("/control/debug/lookahead_point", 1);
  pub_debug_ = create_publisher<String>("/pure_pursuit/debug", 1);
  pub_tracking_status_ = create_publisher<ControllerTrackingStatus>(
      "output/controller_tracking_status", 1);
  pub_command_envelope_ = create_publisher<ControllerCommandEnvelope>(
      "output/controller_command_envelope", 1);
  pub_execution_envelope_ = create_publisher<ControllerExecutionEnvelope>(
      "output/controller_execution_envelope", 1);
  pub_free_run_execution_ack_ =
      create_publisher<FreeRunExecutionAck>("output/free_run_execution_ack", 1);
  pub_free_run_source_key_ =
      create_publisher<FreeRunSourceKey>("output/free_run_source_key", 1);

  const auto bv_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
      "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) {
        PP_DIAGNOSTIC_SCOPE(timing_diagnostic::CallbackKind::kKinematics);
        odometry_ = msg;
        last_odometry_receive_sec_ = steadyNowSec();
      });
  sub_trajectory_ = create_subscription<Trajectory>(
      "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) {
        PP_DIAGNOSTIC_SCOPE(timing_diagnostic::CallbackKind::kTrajectory);
        trajectory_ = msg;
        if (++reference_source_generation_ == 0U) {
          ++reference_source_generation_;
        }
        updateFreeRunSourceKey(*msg);
        if (free_run_source_key_valid_ && pub_free_run_source_key_) {
          pub_free_run_source_key_->publish(free_run_source_key_);
        }
        last_trajectory_receive_sec_ = steadyNowSec();
      });
  sub_mpc_predicted_horizon_ = create_subscription<Trajectory>(
      "input/mpc_predicted_horizon", bv_qos,
      [this](const Trajectory::SharedPtr msg) {
        PP_DIAGNOSTIC_SCOPE(
            timing_diagnostic::CallbackKind::kMpcPredictedHorizon);
        mpc_predicted_horizon_ = msg;
        if (++mpc_source_generation_ == 0U) {
          ++mpc_source_generation_;
        }
        last_mpc_predicted_horizon_receive_sec_ = steadyNowSec();
      });
  sub_mpc_predicted_horizon_contract_ = create_subscription<String>(
      "input/mpc_predicted_horizon_contract", bv_qos,
      [this](const String::SharedPtr msg) {
        PP_DIAGNOSTIC_SCOPE(
            timing_diagnostic::CallbackKind::kMpcPredictedHorizonContract);
        onMpcPredictedHorizonContract(msg);
      });
  sub_overtake_override_ = create_subscription<Float32MultiArray>(
      "input/overtake_reference_override", rclcpp::QoS(1),
      [this](const Float32MultiArray::SharedPtr msg) {
        PP_DIAGNOSTIC_SCOPE(timing_diagnostic::CallbackKind::kOvertakeOverride);
        onOvertakeOverride(msg);
      });
  sub_steering_status_ = create_subscription<SteeringReport>(
      "input/steering_status", bv_qos,
      [this](const SteeringReport::SharedPtr msg) {
        PP_DIAGNOSTIC_SCOPE(timing_diagnostic::CallbackKind::kSteeringStatus);
        latest_steering_status_rad_ = msg->steering_tire_angle;
        last_steering_status_receive_sec_ = steadyNowSec();
      });
  sub_mpc_health_ = create_subscription<String>(
      "input/mpc_health", bv_qos, [this](const String::SharedPtr msg) {
        PP_DIAGNOSTIC_SCOPE(timing_diagnostic::CallbackKind::kMpcHealth);
        onMpcHealth(msg);
      });
  sub_recovery_status_ = create_subscription<RecoveryStatus>(
      "input/recovery_status", bv_qos,
      [this](const RecoveryStatus::SharedPtr msg) {
        PP_DIAGNOSTIC_SCOPE(timing_diagnostic::CallbackKind::kRecoveryStatus);
        onRecoveryStatus(msg);
      });
  sub_overtake_plan_ = create_subscription<OvertakePlan>(
      "input/overtake_plan", bv_qos, [this](const OvertakePlan::SharedPtr msg) {
        PP_DIAGNOSTIC_SCOPE(timing_diagnostic::CallbackKind::kOvertakePlan);
        const builtin_interfaces::msg::Time now_stamp = get_clock()->now();
        const double receive_steady_sec = steadyNowSec();
        if (!overtake_plan_store_.accept(msg, now_stamp, receive_steady_sec)) {
          return;
        }
        overtake_plan_ = msg;
        last_overtake_plan_receive_sec_ = receive_steady_sec;
        last_overtake_plan_stamp_ = msg->header.stamp;
        last_overtake_plan_generation_ = msg->plan_generation;
        updateFreeRunPlanKey(*msg);
      });

  state_lattice_v2_live_proposal_accept_enabled_ = declare_parameter<bool>(
      "state_lattice_v2_live_proposal_accept_enabled", false);
  state_lattice_v2_command_activation_enabled_ = declare_parameter<bool>(
      "state_lattice_v2_command_activation_enabled", false);
  state_lattice_v2_expected_producer_instance_id_ =
      declare_parameter<std::string>(
          "state_lattice_v2_expected_producer_instance_id", "");
  state_lattice_v2_base_attestation_publish_enabled_ = declare_parameter<bool>(
      "state_lattice_v2_base_attestation_publish_enabled", false);
  state_lattice_v2_base_attestation_producer_instance_id_ =
      declare_parameter<std::string>(
          "state_lattice_v2_base_attestation_producer_instance_id", "");
  state_lattice_v2_base_attestation_session_id_ =
      declare_parameter<std::string>(
          "state_lattice_v2_base_attestation_session_id", "");
  if (state_lattice_v2_live_proposal_accept_enabled_ && !recovery_mode_ &&
      validPositiveDecimalId(state_lattice_v2_expected_producer_instance_id_) &&
      state_lattice_v2_base_attestation_publish_enabled_ &&
      !state_lattice_v2_base_attestation_producer_instance_id_.empty() &&
      validPositiveDecimalId(state_lattice_v2_base_attestation_session_id_)) {
    state_lattice_v2_binding_store_ = std::make_unique<
        overtake_transport_contract::state_lattice_v2::BindingStore>(
        state_lattice_v2_expected_producer_instance_id_,
        state_lattice_v2_base_attestation_producer_instance_id_,
        state_lattice_v2_base_attestation_session_id_);
    const auto v2_transport_qos =
        rclcpp::QoS(rclcpp::KeepLast(8)).reliable().durability_volatile();
    const auto v2_status_qos =
        rclcpp::QoS(
            rclcpp::KeepLast(
                overtake_transport_contract::state_lattice_v2::kStatusQosDepth))
            .reliable()
            .durability_volatile();
    pub_state_lattice_v2_binding_status_ =
        create_publisher<StateLatticeV2BindingStatus>(
            "/debug/overtake/state_lattice/v2_binding_status", v2_status_qos);
    pub_state_lattice_v2_base_attestation_ =
        create_publisher<StateLatticeV2BaseAttestation>(
            "/control/overtake/state_lattice/v2_base_attestation",
            v2_transport_qos);
    sub_state_lattice_v2_proposal_ =
        create_subscription<AuthorizedCartesianTrajectoryV2>(
            "/planning/overtake/state_lattice/v2_proposal", v2_transport_qos,
            [this](const AuthorizedCartesianTrajectoryV2::SharedPtr message) {
              // Callback is transport-only: bounded enqueue plus local receipt
              // telemetry.  Validation and uptake happen at control-cycle
              // entry, and this payload is never exposed to command logic.
              overtake_transport_contract::state_lattice_v2::RejectReason
                  reason{};
              (void)state_lattice_v2_binding_store_->enqueue(
                  *message, steadyNowNanoseconds(), &reason);
            });
  } else if (state_lattice_v2_live_proposal_accept_enabled_ ||
             state_lattice_v2_command_activation_enabled_) {
    state_lattice_v2_live_proposal_accept_enabled_ = false;
    RCLCPP_WARN(get_logger(),
                "State Lattice V2 uptake disabled: recovery PP or incomplete "
                "planner/PP attestation identity contract");
    state_lattice_v2_base_attestation_publish_enabled_ = false;
    state_lattice_v2_command_activation_enabled_ = false;
  }

  c002ay1_prod_measure_enabled_ =
      declare_parameter<bool>("c002ay1_prod_measure_enabled", false);
  const auto c002ay1_socket_path =
      declare_parameter<std::string>("c002ay1_prod_measure_socket_path", "");
  const auto c002ay1_run_id =
      declare_parameter<std::string>("c002ay1_prod_measure_run_id", "");
  const auto c002ay1_session_nonce =
      declare_parameter<std::int64_t>("c002ay1_prod_measure_session_nonce", 0);
  const auto c002ay1_instance_id =
      declare_parameter<std::int64_t>("c002ay1_prod_measure_instance_id", 0);
  if (recovery_mode_ && c002ay1_prod_measure_enabled_) {
    c002ay1_prod_measure_enabled_ = false;
  }
  overtake_transport_contract::c002ay1::RuntimeObserverConfig
      c002ay1_observer_config;
  c002ay1_observer_config.enabled =
      c002ay1_prod_measure_enabled_ && !recovery_mode_ &&
      c002ay1_session_nonce > 0 && c002ay1_instance_id > 0;
  c002ay1_observer_config.role =
      overtake_transport_contract::c002ay1::ProducerRole::kPrimaryPurePursuit;
  c002ay1_observer_config.socket_path = c002ay1_socket_path;
  c002ay1_observer_config.run_id = c002ay1_run_id;
  c002ay1_observer_config.session_nonce =
      c002ay1_session_nonce > 0
          ? static_cast<std::uint64_t>(c002ay1_session_nonce)
          : 0U;
  c002ay1_observer_config.producer_instance_id =
      c002ay1_instance_id > 0 ? static_cast<std::uint64_t>(c002ay1_instance_id)
                              : 0U;
  c002ay1_runtime_observer_ =
      overtake_transport_contract::c002ay1::RuntimeObservationWriter::attach(
          c002ay1_observer_config);

  c002ay0_controller_implementation_digest_ =
      aw2ControllerAdapterImplementationDigestV1();
  c002ay0_controller_config_digest_ = aw2ControllerAdapterConfigDigestV1(
      {wheel_base_, lookahead_gain_, lookahead_min_distance_,
       speed_proportional_gain_, steering_tire_angle_gain_,
       pp_control_delay_sec_, pp_prediction_dt_sec_,
       steering_time_constant_sec_, horizon_curvature_feedforward_gain_,
       horizon_curvature_feedforward_max_rad_,
       free_run_live_exact_hard_steering_limit_rad_,
       free_run_live_exact_hard_steering_rate_limit_radps_,
       steering_command_nominal_dt_sec_},
      {use_external_target_vel_, use_mpc_predicted_horizon_,
       use_overtake_reference_override_,
       curvature_adaptive_lookahead_enabled_});

  c002ay0_shadow_capture_enabled_ =
      declare_parameter<bool>("c002ay0_shadow_capture_enabled", false);
  state_lattice_source_binding_shadow_enabled_ = declare_parameter<bool>(
      "state_lattice_source_binding_shadow_enabled", false);
  const auto c002ay0_worker_path = declare_parameter<std::string>(
      "c002ay0_shadow_worker_path", ay0ShadowWorkerPath());
  const auto c002ay0_generation =
      declare_parameter<std::int64_t>("c002ay0_shadow_session_generation", 0);
  const auto c002ay0_nonce =
      declare_parameter<std::int64_t>("c002ay0_shadow_session_nonce", 0);
  if (recovery_mode_ && c002ay0_shadow_capture_enabled_) {
    c002ay0_shadow_capture_enabled_ = false;
  }
  if (c002ay0_shadow_capture_enabled_ && c002ay0_generation > 0 &&
      c002ay0_nonce > 0 && !c002ay0_worker_path.empty()) {
    c002ay0_session_generation_ =
        static_cast<std::uint64_t>(c002ay0_generation);
    c002ay0_session_nonce_ = static_cast<std::uint64_t>(c002ay0_nonce);
    overtake_transport_contract::c002ay0::FixedBaseWorkerConfig config;
    config.enabled = true;
    config.executable_path = c002ay0_worker_path;
    config.session_generation = c002ay0_session_generation_;
    config.session_nonce = c002ay0_session_nonce_;
    config.controller_instance_id = producer_instance_id_;
    config.source_binding_enabled =
        state_lattice_source_binding_shadow_enabled_;
    config.use_sim_time = get_parameter("use_sim_time").as_bool();
    c002ay0_worker_session_ =
        overtake_transport_contract::c002ay0::FixedBaseWorkerSession::start(
            config);
    if (c002ay0_worker_session_ == nullptr ||
        !c002ay0_worker_session_->ready()) {
      c002ay0_shadow_capture_enabled_ = false;
      RCLCPP_WARN(get_logger(),
                  "AY0 shadow capture unavailable; motion path continues");
    }
  } else if (c002ay0_shadow_capture_enabled_) {
    RCLCPP_WARN(
        get_logger(),
        "AY0 shadow capture disabled: generation/nonce/path is invalid");
    c002ay0_shadow_capture_enabled_ = false;
  }
  if (state_lattice_source_binding_shadow_enabled_ &&
      !c002ay0_shadow_capture_enabled_) {
    RCLCPP_WARN(get_logger(), "State Lattice source-binding shadow disabled: "
                              "AY0 base capture is unavailable");
    state_lattice_source_binding_shadow_enabled_ = false;
  }
  const auto race_arm_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  sub_state_lattice_shadow_race_armed_ = create_subscription<Bool>(
      "input/race_armed", race_arm_qos, [this](const Bool::SharedPtr message) {
        PP_DIAGNOSTIC_SCOPE(timing_diagnostic::CallbackKind::kRaceArmed);
        if (c002ay0_shadow_capture_enabled_ ||
            state_lattice_v2_base_attestation_publish_enabled_) {
          state_lattice_shadow_race_arm_epoch_.observe(message->data);
        }
      });

  aw2_shadow_transport_enabled_ =
      declare_parameter<bool>("aw2_shadow_transport_enabled", true);
  if (aw2_shadow_transport_enabled_ && !recovery_mode_) {
    const auto supervisor_path = siblingShadowSupervisorPath();
    aw2_shadow_session_ = aw2_shadow::AsyncProducerSession::start(
        supervisor_path, producer_instance_id_, producer_instance_id_,
        c002ay0_controller_implementation_digest_,
        c002ay0_controller_config_digest_, std::chrono::milliseconds(250));
    if (aw2_shadow_session_ == nullptr) {
      RCLCPP_WARN(get_logger(),
                  "AW2 shadow transport unavailable; motion path continues");
    }
  }

  using namespace std::literals::chrono_literals;
  timer_ =
      create_wall_timer(10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp) {
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.longitudinal.jerk = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  cmd.lateral.steering_tire_rotation_rate = 0.0;
  return cmd;
}

void SimplePurePursuit::updateFreeRunPlanKey(const OvertakePlan &plan) {
  free_run_plan_key_valid_ = false;
  free_run_plan_key_ = FreeRunPlanKey();
  if (!free_run_live_exact_ack_enabled_ || plan.plan_generation == 0U ||
      plan.planner_instance_id == 0U || plan.race_arm_epoch == 0U ||
      plan.header.stamp.sec < 0 || plan.header.stamp.nanosec >= 1000000000U) {
    return;
  }
  if (!overtake_transport_contract::c002ay0::validateFreeRunPlanIdentityV1(
          plan)) {
    return;
  }
  free_run_plan_key_.canonical_algorithm_version =
      FreeRunPlanKey::CANONICAL_ALGORITHM_V1;
  free_run_plan_key_.race_arm_epoch = plan.race_arm_epoch;
  free_run_plan_key_.planner_instance_id = plan.planner_instance_id;
  free_run_plan_key_.plan_generation = plan.plan_generation;
  free_run_plan_key_.plan_stamp = plan.header.stamp;
  std::copy(plan.free_run_canonical_payload_sha256.begin(),
            plan.free_run_canonical_payload_sha256.end(),
            free_run_plan_key_.canonical_plan_payload_sha256.begin());
  free_run_plan_key_valid_ = true;
}

void SimplePurePursuit::updateFreeRunSourceKey(const Trajectory &trajectory) {
  free_run_source_key_valid_ = false;
  free_run_source_key_ = FreeRunSourceKey();
  if ((!free_run_live_exact_ack_enabled_ && !pp_core_exact_snapshot_enabled_) ||
      reference_source_generation_ == 0U || trajectory.header.stamp.sec < 0 ||
      trajectory.header.stamp.nanosec >= 1000000000U) {
    return;
  }
  const auto canonical =
      overtake_transport_contract::c002ay0::canonicalizeFreeRunReferenceV1(
          trajectory);
  if (!canonical.valid()) {
    return;
  }
  free_run_source_key_.canonical_algorithm_version =
      FreeRunSourceKey::CANONICAL_ALGORITHM_V1;
  // The upstream trajectory publisher does not expose a stable producer ID.
  // Bind this identity to the immutable reference cache owned by this PP
  // process; source_generation and source_stamp distinguish its deliveries.
  free_run_source_key_.baseline_instance_id = producer_instance_id_;
  free_run_source_key_.controller_instance_id = producer_instance_id_;
  free_run_source_key_.source_generation = reference_source_generation_;
  free_run_source_key_.source_stamp = trajectory.header.stamp;
  free_run_source_key_.original_point_count =
      static_cast<std::uint32_t>(trajectory.points.size());
  std::copy(canonical.sha256.begin(), canonical.sha256.end(),
            free_run_source_key_.baseline_reference_sha256.begin());
  std::copy(c002ay0_controller_implementation_digest_.begin(),
            c002ay0_controller_implementation_digest_.end(),
            free_run_source_key_.controller_implementation_sha256.begin());
  std::copy(c002ay0_controller_config_digest_.begin(),
            c002ay0_controller_config_digest_.end(),
            free_run_source_key_.controller_config_sha256.begin());
  free_run_source_key_valid_ = true;
}

void SimplePurePursuit::publishFreeRunExecutionAck(
    const ControllerCommandEnvelope &command_envelope,
    const ControllerTrackingStatus &tracking_status,
    const ControlTrajectoryContext &context,
    const ControlPosePrediction &control_pose,
    const AckermannControlCommand &raw_command,
    double required_spatial_horizon_m, std::size_t speed_cap_trajectory_index,
    std::size_t curvature_last_read_trajectory_index,
    std::size_t lookahead_selected_trajectory_index,
    std::size_t required_horizon_end_trajectory_index,
    bool lookahead_endpoint_fallback) {
  if (!free_run_live_exact_ack_enabled_ || !pub_free_run_execution_ack_ ||
      !free_run_plan_key_valid_ || !free_run_source_key_valid_ ||
      context.base_trajectory == nullptr || context.trajectory == nullptr ||
      context.mpc_horizon_applied || context.overtake_override_applied ||
      context.source != "trajectory" || overtake_plan_ == nullptr ||
      overtake_plan_->header.stamp != free_run_plan_key_.plan_stamp ||
      overtake_plan_->plan_generation != free_run_plan_key_.plan_generation ||
      context.base_trajectory->header.stamp !=
          free_run_source_key_.source_stamp ||
      reference_source_generation_ != free_run_source_key_.source_generation) {
    return;
  }

  FreeRunExecutionAck ack;
  ack.header = command_envelope.header;
  ack.schema_version = FreeRunExecutionAck::SCHEMA_V2;
  ack.ack_eligible = false;
  ack.evidence_state = FreeRunExecutionAck::EVIDENCE_MISSING_INPUT;
  ack.evidence_reason = "incomplete";
  ack.plan_key = free_run_plan_key_;
  ack.source_key = free_run_source_key_;
  ack.controller_sequence = command_envelope.command_sequence;
  ack.controller_command_stamp = command_envelope.command.stamp;
  ack.control_pose.position = control_pose.position;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, control_pose.yaw);
  ack.control_pose.orientation.x = orientation.x();
  ack.control_pose.orientation.y = orientation.y();
  ack.control_pose.orientation.z = orientation.z();
  ack.control_pose.orientation.w = orientation.w();
  ack.control_pose_stamp = command_envelope.command.stamp;
  ack.nearest_trajectory_index =
      static_cast<std::uint32_t>(context.nearest_index);
  ack.trajectory_progress_m = 0.0;
  ack.raw_controller_command = raw_command;
  ack.output_controller_command = command_envelope.command;
  ack.raw_controller_command_sha256 =
      canonicalControllerCommandDigestV1(raw_command);
  ack.output_controller_command_sha256 =
      canonicalControllerCommandDigestV1(command_envelope.command);
  ack.raw_steering_tire_angle_rad = raw_command.lateral.steering_tire_angle;
  ack.output_steering_tire_angle_rad =
      command_envelope.command.lateral.steering_tire_angle;
  ack.hard_steering_tire_angle_limit_rad =
      free_run_live_exact_hard_steering_limit_rad_;
  ack.required_spatial_horizon_m = required_spatial_horizon_m;

  const auto base = buildControllerGeometryV1(
      *context.base_trajectory, free_run_source_key_.original_point_count,
      context.nearest_index, speed_cap_trajectory_index,
      curvature_last_read_trajectory_index, lookahead_selected_trajectory_index,
      required_horizon_end_trajectory_index, lookahead_endpoint_fallback,
      required_spatial_horizon_m);
  std::optional<ControllerGeometryBuildResult> applied_build;
  if (context.trajectory != context.base_trajectory) {
    applied_build.emplace(buildControllerGeometryV1(
        *context.trajectory, free_run_source_key_.original_point_count,
        context.nearest_index, speed_cap_trajectory_index,
        curvature_last_read_trajectory_index,
        lookahead_selected_trajectory_index,
        required_horizon_end_trajectory_index, lookahead_endpoint_fallback,
        required_spatial_horizon_m));
  }
  const auto &applied =
      applied_build.has_value() ? applied_build.value() : base;
  ack.base_geometry = base.geometry;
  ack.applied_geometry = applied.geometry;
  ack.available_spatial_horizon_m = std::min(
      base.geometry.total_arc_length_m, applied.geometry.total_arc_length_m);
  if (base.valid && applied.valid) {
    ack.base_geometry.full_source_digest_state =
        ack.base_geometry.FULL_SOURCE_DIGEST_COMPLETE;
    ack.applied_geometry.full_source_digest_state =
        ack.applied_geometry.FULL_SOURCE_DIGEST_COMPLETE;
    ack.base_geometry.full_source_sha256 =
        free_run_source_key_.baseline_reference_sha256;
    ack.applied_geometry.full_source_sha256 =
        free_run_source_key_.baseline_reference_sha256;
  }

  const bool command_digest_valid =
      std::any_of(ack.raw_controller_command_sha256.begin(),
                  ack.raw_controller_command_sha256.end(),
                  [](std::uint8_t value) { return value != 0U; }) &&
      std::any_of(ack.output_controller_command_sha256.begin(),
                  ack.output_controller_command_sha256.end(),
                  [](std::uint8_t value) { return value != 0U; });
  const bool actuator_valid =
      std::isfinite(free_run_live_exact_hard_steering_limit_rad_) &&
      free_run_live_exact_hard_steering_limit_rad_ > 0.0 &&
      std::isfinite(free_run_live_exact_hard_steering_rate_limit_radps_) &&
      free_run_live_exact_hard_steering_rate_limit_radps_ > 0.0 &&
      std::isfinite(ack.raw_steering_tire_angle_rad) &&
      std::isfinite(ack.output_steering_tire_angle_rad) &&
      std::abs(ack.output_steering_tire_angle_rad) <=
          free_run_live_exact_hard_steering_limit_rad_ + 1.0e-6 &&
      std::isfinite(
          ack.output_controller_command.lateral.steering_tire_rotation_rate) &&
      std::abs(
          ack.output_controller_command.lateral.steering_tire_rotation_rate) <=
          free_run_live_exact_hard_steering_rate_limit_radps_ + 1.0e-6;
  const bool horizon_valid =
      std::isfinite(required_spatial_horizon_m) &&
      required_spatial_horizon_m > 0.0 &&
      std::isfinite(ack.available_spatial_horizon_m) &&
      ack.available_spatial_horizon_m >= required_spatial_horizon_m &&
      !lookahead_endpoint_fallback;
  if (!base.bounded || !applied.bounded) {
    ack.evidence_state = FreeRunExecutionAck::EVIDENCE_RESOURCE_LIMIT_EXCEEDED;
    ack.evidence_reason = "used_interval_unbounded";
  } else if (!base.valid || !applied.valid || !command_digest_valid) {
    ack.evidence_state = FreeRunExecutionAck::EVIDENCE_GEOMETRY_INVALID;
    ack.evidence_reason = "geometry_or_command_invalid";
  } else if (!horizon_valid) {
    ack.evidence_state = FreeRunExecutionAck::EVIDENCE_HORIZON_INSUFFICIENT;
    ack.evidence_reason = "spatial_horizon_insufficient";
  } else if (!actuator_valid) {
    ack.evidence_state = FreeRunExecutionAck::EVIDENCE_ACTUATOR_LIMIT_INVALID;
    ack.evidence_reason = "hard_steering_limit_exceeded";
  } else if (!tracking_status.trajectory_tracking_usable ||
             !tracking_status.pp_command_fresh) {
    ack.evidence_state = FreeRunExecutionAck::EVIDENCE_MISSING_INPUT;
    ack.evidence_reason = "tracking_not_usable";
  } else {
    ack.ack_eligible = true;
    ack.evidence_state = FreeRunExecutionAck::EVIDENCE_COMPLETE;
    ack.evidence_reason = "complete";
    ack.ack_sha256 = canonicalFreeRunExecutionAckDigestV2(ack);
    if (!std::any_of(ack.ack_sha256.begin(), ack.ack_sha256.end(),
                     [](std::uint8_t value) { return value != 0U; })) {
      ack.ack_eligible = false;
      ack.evidence_state = FreeRunExecutionAck::EVIDENCE_GEOMETRY_INVALID;
      ack.evidence_reason = "canonical_v2_invalid";
    }
  }
  pub_free_run_execution_ack_->publish(ack);
}

void SimplePurePursuit::applyStateLatticeV2CycleResult(
    const overtake_transport_contract::state_lattice_v2::CycleResult &result) {
  if (result.run_invalid) {
    // BindingStore clears its exact base history on clock recovery and makes
    // overflow terminal. Keep the republish cache in the same fail-closed
    // lifecycle so a key no longer recorded by the Store cannot be reused.
    state_lattice_v2_last_published_base_attestation_.reset();
    state_lattice_v2_base_attestation_refresh_pending_ = false;
  }
  if (state_lattice_v4_poc_identity_gate_enabled_ &&
      result.availability_present && result.availability_identity.has_value()) {
    state_lattice_v4_poc_identity_ = result.availability_identity;
  } else {
    state_lattice_v4_poc_identity_.reset();
  }
  // Keep an exact, availability-bound Cartesian cache whenever the typed
  // proposal channel is enabled. Global V2 command activation remains a
  // separate experimental mode; V4 may consume this cache only after its
  // own spatial authority heartbeat and identity checks succeed.
  if (result.accepted.has_value()) {
    const auto converted =
        stateLatticeV2ProposalToTrajectory(result.accepted.value());
    if (converted.has_value()) {
      StateLatticeV2ControlTrajectoryCache cache;
      cache.identity = result.accepted->identity;
      cache.proposal =
          std::make_shared<const AuthorizedCartesianTrajectoryV2>(
              result.accepted.value());
      cache.trajectory =
          std::make_shared<Trajectory>(std::move(converted.value()));
      cache.receive_steady_sec = steadyNowSec();
      state_lattice_v2_control_trajectory_cache_ = std::move(cache);
    } else {
      state_lattice_v2_control_trajectory_cache_.reset();
    }
  }
  const bool current_v2_rejected_this_cycle =
      stateLatticeV2CurrentAvailabilityRejected(result);
  const bool exact_cached_availability =
      !current_v2_rejected_this_cycle && result.availability_present &&
      result.availability_identity.has_value() &&
      state_lattice_v2_control_trajectory_cache_.has_value() &&
      state_lattice_v2_control_trajectory_cache_->trajectory != nullptr &&
      stateLatticeV2IdentityMatches(
          state_lattice_v2_control_trajectory_cache_->identity,
          result.availability_identity.value());
  if (exact_cached_availability) {
    if (result.accepted.has_value()) {
      // The proposal is accepted, converted, installed, and is the exact
      // current availability. Ratchet at most once on the next normal,
      // fully-validated publication; timer cycles alone never renew it.
      state_lattice_v2_base_attestation_refresh_pending_ = true;
    }
    (void)activatePendingStateLatticeV4Contract(
        result.availability_identity.value());
  }
  if (!exact_cached_availability) {
    state_lattice_v2_control_trajectory_cache_.reset();
  }
}

void SimplePurePursuit::onTimer() {
  const auto stamp = get_clock()->now();
  const double now_sec = steadyNowSec();
  if (state_lattice_v2_live_proposal_accept_enabled_) {
    const builtin_interfaces::msg::Time stamp_msg = stamp;
    const auto v2_result = state_lattice_v2_binding_store_->beginCycle(
        stamp_msg, steadyNowNanoseconds());
    publishStateLatticeV2BindingStatus(stamp_msg, v2_result);
    applyStateLatticeV2CycleResult(v2_result);
  }
  const auto plan_snapshot = overtake_plan_store_.beginControlCycle();
  const builtin_interfaces::msg::Time stamp_msg = stamp;
  overtake_transport_contract::c002ay1::PpObservationScope c002ay1_observation(
      c002ay1_runtime_observer_, stamp_msg.sec, stamp_msg.nanosec);

  // 1. 入力が古い場合は、制御計算へ進まず停止/diagnosticだけを出す。
  const auto freshness = evaluateInputFreshness(now_sec);
  if (handleInvalidFreshness(stamp, freshness)) {
    publishControllerTrackingStatus(stamp, nullptr, nullptr, nullptr, nullptr,
                                    now_sec, plan_snapshot.get());
    c002ay1_observation.record().flags |=
        overtake_transport_contract::c002ay1::kFlagEmitted;
    c002ay1_observation.setReturnReason(
        overtake_transport_contract::c002ay1::ReturnReason::kEarlyInputStale);
    return;
  }

  // Command activation is opt-in and can only consume the exact BindingStore
  // availability cohort selected above.  A reject, expiry, or conversion
  // failure therefore takes the established stale-input stop path; it never
  // falls back to the baseline trajectory or legacy override stream.
  if (state_lattice_v2_command_activation_enabled_ &&
      !state_lattice_v2_control_trajectory_cache_.has_value()) {
    const bool rendezvous_pending = stateLatticeV4RendezvousPending();
    FreshnessResult invalid;
    invalid.ages = freshness.ages;
    invalid.reason = rendezvous_pending
                         ? "override_contract_missing_or_stale"
                         : "state_lattice_v2_unavailable";
    publishStaleDebug(stamp, invalid);
    if (!rendezvous_pending) {
      resetSteeringLimiter();
    }
    if (stop_on_stale_input_ ||
        state_lattice_v4_poc_command_activation_enabled_) {
      publishStopForStaleInput(stamp, invalid, true);
    }
    publishControllerTrackingStatus(stamp, nullptr, nullptr, nullptr, nullptr,
                                    now_sec, plan_snapshot.get(),
                                    rendezvous_pending);
    c002ay1_observation.record().flags |=
        overtake_transport_contract::c002ay1::kFlagEmitted;
    c002ay1_observation.setReturnReason(
        overtake_transport_contract::c002ay1::ReturnReason::kEarlyInputStale);
    return;
  }

  // A State Lattice -> Pure Pursuit live profile treats the override stream as
  // an authority heartbeat. If it is missing or stale, do not silently fall
  // back to the baseline trajectory.
  const double override_contract_age_sec =
      inputAgeSec(last_valid_override_contract_sec_, now_sec);
  const bool override_contract_fresh =
      use_overtake_reference_override_ &&
      last_valid_override_contract_received_ &&
      ageFresh(override_contract_age_sec, overtake_override_timeout_sec_) &&
      ageFresh(override_contract_age_sec, max_override_age_sec_);
  const bool v4_identity_required =
      state_lattice_v4_poc_identity_gate_enabled_ &&
      overtake_lateral_override_active_ &&
      !overtake_longitudinal_offsets_m_.empty();
  const bool v4_identity_matches =
      !v4_identity_required ||
      (state_lattice_v4_poc_command_activation_enabled_
           ? v4PocCommandContractReady(now_sec)
           : (trajectory_ != nullptr &&
              state_lattice_v4_poc_identity_.has_value() &&
              v4PocIdentityMatches(
                  state_lattice_v4_poc_identity_.value(),
                  overtake_override_generation_, trajectory_->header.frame_id,
                  trajectory_->header.stamp, reference_source_generation_)));
  const bool v2_authority_fresh =
      state_lattice_v2_command_activation_enabled_ &&
      state_lattice_v2_control_trajectory_cache_.has_value();
  if (state_lattice_v4_poc_command_activation_enabled_ &&
      (state_lattice_v2_command_activation_enabled_ ||
       !v4PocCommandContractAvailable(now_sec))) {
    const bool rendezvous_pending = stateLatticeV4RendezvousPending();
    FreshnessResult invalid;
    invalid.ages = freshness.ages;
    invalid.ages.override_age_sec = override_contract_age_sec;
    invalid.reason = rendezvous_pending
                         ? "override_contract_missing_or_stale"
                         : "state_lattice_v4_unavailable";
    publishStaleDebug(stamp, invalid);
    // V2 geometry and its V4 authority heartbeat arrive independently. No
    // command is emitted while only one half is present, but a fresh newer
    // half may retain the last published steering reference so an otherwise
    // continuous acquisition is not restarted by callback ordering. Exact
    // identity is still required before activation; stale or same-generation
    // evidence falls through to the established reset.
    if (!rendezvous_pending) {
      resetSteeringLimiter();
    }
    publishStopForStaleInput(stamp, invalid, true);
    publishControllerTrackingStatus(stamp, nullptr, nullptr, nullptr, nullptr,
                                    now_sec, plan_snapshot.get(),
                                    rendezvous_pending);
    c002ay1_observation.record().flags |=
        overtake_transport_contract::c002ay1::kFlagEmitted;
    c002ay1_observation.setReturnReason(
        overtake_transport_contract::c002ay1::ReturnReason::kEarlyInputStale);
    return;
  }
  if (require_overtake_reference_override_fresh_ &&
      !v2_authority_fresh &&
      (!override_contract_fresh || !v4_identity_matches)) {
    FreshnessResult invalid;
    invalid.ages = freshness.ages;
    invalid.ages.override_age_sec = override_contract_age_sec;
    invalid.reason =
        !v4_identity_matches
            ? "required_overtake_override_identity_mismatch"
            : (last_valid_override_contract_received_
                   ? "required_overtake_override_stale"
                   : "required_overtake_override_missing_or_invalid");
    publishStaleDebug(stamp, invalid);
    resetSteeringLimiter();
    if (stop_on_stale_input_ ||
        state_lattice_v4_poc_command_activation_enabled_) {
      publishStopForStaleInput(stamp, invalid);
    }
    publishControllerTrackingStatus(stamp, nullptr, nullptr, nullptr, nullptr,
                                    now_sec, plan_snapshot.get());
    c002ay1_observation.record().flags |=
        overtake_transport_contract::c002ay1::kFlagEmitted;
    c002ay1_observation.setReturnReason(
        overtake_transport_contract::c002ay1::ReturnReason::kEarlyInputStale);
    return;
  }

  // 2. 古いovertake overrideはこの周期の参照生成へ混ぜない。
  clearStaleOvertakeOverride(now_sec);
  const auto control_pose = predictControlPose(now_sec);
  const auto mpc_horizon_freshness = evaluateMpcPredictedHorizon(now_sec);

  // 3. 通常trajectory / MPC horizon / overtake override適用後trajectoryを
  //    1つのcontrol trajectoryへ正規化する。
  const auto context =
      selectControlTrajectory(control_pose, now_sec, mpc_horizon_freshness);
  if (!context.valid || context.trajectory == nullptr ||
      context.trajectory->points.empty()) {
    FreshnessResult invalid;
    invalid.reason = context.invalid_reason;
    publishStaleDebug(stamp, invalid);
    resetSteeringLimiter();
    if (stop_on_stale_input_ ||
        state_lattice_v4_poc_command_activation_enabled_) {
      publishStopForStaleInput(stamp, invalid);
    }
    publishControllerTrackingStatus(stamp, nullptr, nullptr, nullptr, nullptr,
                                    now_sec, plan_snapshot.get());
    c002ay1_observation.record().flags |=
        overtake_transport_contract::c002ay1::kFlagEmitted;
    c002ay1_observation.setReturnReason(
        overtake_transport_contract::c002ay1::ReturnReason::kEarlyInputInvalid);
    return;
  }

  AckermannControlCommand cmd = zeroAckermannControlCommand(stamp);
  const auto &nearest_traj_point =
      context.trajectory->points.at(context.nearest_index);

  // 4. 速度と横制御を別々に計算し、最後にAckermann commandへ詰める。
  const auto longitudinal = computeLongitudinalCommand(context, now_sec);
  cmd.longitudinal.speed = longitudinal.target_speed_mps;
  cmd.longitudinal.acceleration = longitudinal.acceleration_mps2;

  const auto lateral =
      computeLateralCommand(context, control_pose, longitudinal);
  publishLookaheadPoint(lateral.lookahead_point_x, lateral.lookahead_point_y,
                        nearest_traj_point.pose.position.z);
  cmd.lateral.steering_tire_angle = lateral.steering_tire_angle_rad;
  // ControllerCommandEnvelope schema v1ではrotation_rate fieldは常に0。
  // 実レート制限は周期間のbounded angle差分で適用し、Muxが独立に再検証する。
  cmd.lateral.steering_tire_rotation_rate = 0.0;
  auto raw_cmd = cmd;
  raw_cmd.lateral.steering_tire_angle =
      lateral.requested_output_steering_tire_angle_rad;
  raw_cmd.lateral.steering_tire_rotation_rate = 0.0;

  publishDebug(
      cmd.stamp, *context.trajectory, context.nearest_index,
      longitudinal.target_speed_mps, longitudinal.current_speed_mps,
      longitudinal.acceleration_mps2, lateral.base_lookahead_distance_m,
      lateral.desired_lookahead_distance_m, lateral.lookahead_distance_m,
      lateral.path_curvature_1pm, lateral.signed_path_curvature_1pm,
      lateral.curvature_window_distance_m, lateral.lookahead_point_x,
      lateral.lookahead_point_y, lateral.rear_x, lateral.rear_y,
      lateral.alpha_rad, lateral.pure_pursuit_steering_tire_angle_rad,
      lateral.curvature_feedforward_steering_rad,
      lateral.raw_steering_tire_angle_rad, lateral.steering_tire_angle_rad,
      context.overtake_override_applied, overtakeLateralOffset(0),
      longitudinal.overtake_speed_cap_mps, now_sec, context.mpc_horizon_applied,
      longitudinal.mpc_horizon_velocity_cap_applied,
      longitudinal.mpc_horizon_velocity_cap_mps,
      context.evaluated_horizon_stamp_sec,
      context.evaluated_horizon_stamp_nanosec,
      context.applied_horizon_stamp_sec, context.applied_horizon_stamp_nanosec,
      context.applied_horizon_source, context.applied_horizon_mode_id,
      context.applied_horizon_generation, context.mpc_horizon_freshness,
      context.source, context.overtake_override_apply_reason,
      context.v4_poc_contract, context.v4_poc_identity_required,
      context.v4_poc_identity_matched, context.v4_poc_geometry_applied,
      context.v4_poc_generation, context.overtake_spatial_horizon_arc_m,
      context.overtake_spatial_horizon_required_arc_m, control_pose);

  if (recovery_mode_) {
    std::string recovery_reason;
    if (recoveryStatusAllowsControl(now_sec, *context.trajectory,
                                    &recovery_reason)) {
      publishRecoveryControlCommand(cmd, *context.trajectory);
      commitSteeringLimiterCommand(cmd.lateral.steering_tire_angle,
                                   context.source);
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(),
          static_cast<int>(std::max(0.1, diagnostic_throttle_sec_) * 1000.0),
          "Recovery PurePursuit command blocked: %s", recovery_reason.c_str());
    }
    publishControllerTrackingStatus(stamp, &context, nullptr, &longitudinal,
                                    &lateral, now_sec, plan_snapshot.get());
    c002ay1_observation.setReturnReason(
        overtake_transport_contract::c002ay1::ReturnReason::kCompletedNoEmit);
    return;
  }

  // commandとstatusは同一stampのpairであり、muxは両方が揃うまでfail-closedに
  // する。proofを先にcache可能にして、command publish直後へmux timerが
  // 割り込むだけの一過性command_stamp_mismatchを避ける。
  const auto tracking_status =
      publishControllerTrackingStatus(stamp, &context, &cmd, &longitudinal,
                                      &lateral, now_sec, plan_snapshot.get());
  const auto command_envelope = publishControllerCommandEnvelope(
      cmd, tracking_status, plan_snapshot.get(), &context);
  if (command_envelope.has_value()) {
    publishControllerExecutionEnvelope(command_envelope.value(), &context,
                                       &control_pose, &lateral, now_sec,
                                       plan_snapshot.get());
  }
  pub_cmd_->publish(cmd);
  commitSteeringLimiterCommand(cmd.lateral.steering_tire_angle, context.source);
  pub_raw_cmd_->publish(raw_cmd);
  if (command_envelope.has_value()) {
    double required_spatial_horizon_m = lateral.lookahead_distance_m;
    if (context.overtake_override_applied &&
        std::isfinite(context.overtake_spatial_horizon_required_arc_m) &&
        context.overtake_spatial_horizon_required_arc_m > 0.0) {
      required_spatial_horizon_m =
          std::max(required_spatial_horizon_m,
                   context.overtake_spatial_horizon_required_arc_m);
    }
    const auto required_horizon_end = requiredHorizonEndIndex(
        *context.trajectory, context.nearest_index, required_spatial_horizon_m);
    publishFreeRunExecutionAck(
        command_envelope.value(), tracking_status, context, control_pose,
        raw_cmd, required_spatial_horizon_m,
        longitudinal.speed_cap_trajectory_index,
        lateral.curvature_last_read_trajectory_index,
        lateral.lookahead_selected_trajectory_index,
        required_horizon_end.value_or(context.trajectory->points.size()),
        lateral.lookahead_endpoint_fallback);
    publishStateLatticeV2BaseAttestation(command_envelope.value(), context);
    captureAy0BaseShadow(command_envelope.value(), context);
    captureControllerAppliedShadow(
        command_envelope.value(), &context, &control_pose, raw_cmd,
        required_spatial_horizon_m, longitudinal.speed_cap_trajectory_index,
        lateral.curvature_last_read_trajectory_index,
        lateral.lookahead_selected_trajectory_index,
        required_horizon_end.value_or(context.trajectory->points.size()),
        lateral.lookahead_endpoint_fallback, now_sec);
  }
  c002ay1_observation.record().flags |=
      overtake_transport_contract::c002ay1::kFlagEmitted;
  c002ay1_observation.setReturnReason(
      overtake_transport_contract::c002ay1::ReturnReason::kCompletedEmit);
}

void SimplePurePursuit::publishStateLatticeV2BindingStatus(
    const builtin_interfaces::msg::Time &stamp,
    const overtake_transport_contract::state_lattice_v2::CycleResult &result) {
  if (pub_state_lattice_v2_binding_status_ == nullptr) {
    return;
  }
  const auto populate_base_attestation_diagnostic =
      [this](StateLatticeV2BindingStatus &status) {
        status.base_attestation_stage =
            state_lattice_v2_base_attestation_stage_;
        status.base_attestation_build_result =
            state_lattice_v2_base_attestation_build_result_;
        status.base_attestation_build_diagnostic =
            state_lattice_v2_base_attestation_build_diagnostic_;
        status.base_attestation_validation_reason =
            state_lattice_v2_base_attestation_validation_reason_;
        status.base_attestation_point_invalid_field =
            state_lattice_v2_base_attestation_point_invalid_field_;
        status.base_attestation_failure_window_index =
            state_lattice_v2_base_attestation_failure_window_index_;
        status.base_attestation_failure_source_index =
            state_lattice_v2_base_attestation_failure_source_index_;
        status.base_attestation_snapshot_result =
            state_lattice_v2_base_attestation_snapshot_result_;
        status.base_attestation_local_reject_reason =
            state_lattice_v2_base_attestation_local_reject_reason_;
        status.base_attestation_attempt_count =
            state_lattice_v2_base_attestation_attempt_count_;
        status.base_attestation_publish_count =
            state_lattice_v2_base_attestation_publish_count_;
        status.base_attestation_race_arm_epoch =
            state_lattice_v2_base_attestation_race_arm_epoch_;
      };
  StateLatticeV2BindingStatus summary;
  summary.header.stamp = stamp;
  summary.header.frame_id = "map";
  summary.availability_summary = true;
  summary.pp_cycle_sequence = result.pp_cycle_sequence;
  summary.availability_present = result.availability_present;
  if (result.availability_identity.has_value()) {
    summary.availability_identity = result.availability_identity.value();
  }
  summary.availability_safety_valid_until =
      result.availability_safety_valid_until;
  summary.availability_transition =
      static_cast<std::uint8_t>(result.availability_transition);
  summary.timer_entry_monotonic_ns = result.timer_entry_monotonic_ns;
  summary.run_invalid = result.run_invalid;
  summary.overflow_count = result.overflow_count;
  summary.overflow_first_sequence = result.overflow_first_sequence;
  summary.overflow_last_sequence = result.overflow_last_sequence;
  summary.cycle_event_count = static_cast<std::uint8_t>(result.event_count);
  populate_base_attestation_diagnostic(summary);
  pub_state_lattice_v2_binding_status_->publish(summary);
  for (std::size_t index = 0U; index < result.event_count; ++index) {
    const auto &event = result.events[index];
    StateLatticeV2BindingStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = "map";
    status.availability_summary = false;
    status.pp_cycle_sequence = result.pp_cycle_sequence;
    status.availability_present = result.availability_present;
    if (result.availability_identity.has_value()) {
      status.availability_identity = result.availability_identity.value();
    }
    status.availability_safety_valid_until =
        result.availability_safety_valid_until;
    status.availability_transition =
        static_cast<std::uint8_t>(result.availability_transition);
    status.timer_entry_monotonic_ns = result.timer_entry_monotonic_ns;
    if (event.identity.has_value()) {
      status.identity = event.identity.value();
    }
    status.first_uptake_pass = event.first_uptake;
    status.hold_cycle_index = event.hold_cycle_index;
    status.reject_reason = static_cast<std::uint8_t>(event.reject_reason);
    status.receive_monotonic_ns = event.receive_monotonic_ns;
    status.accepted_monotonic_ns = event.accepted_monotonic_ns;
    status.run_invalid = result.run_invalid;
    status.overflow_count = result.overflow_count;
    status.overflow_first_sequence = result.overflow_first_sequence;
    status.overflow_last_sequence = result.overflow_last_sequence;
    status.cycle_event_index = static_cast<std::uint8_t>(index);
    status.cycle_event_count = static_cast<std::uint8_t>(result.event_count);
    populate_base_attestation_diagnostic(status);
    pub_state_lattice_v2_binding_status_->publish(status);
  }
}

bool SimplePurePursuit::handleInvalidFreshness(
    const rclcpp::Time &stamp, const FreshnessResult &freshness) {
  if (freshness.fresh) {
    return false;
  }

  has_smoothed_lookahead_distance_ = false;
  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(),
      static_cast<int>(std::max(0.1, diagnostic_throttle_sec_) * 1000.0),
      "PurePursuit input invalid: %s odom_age=%.3f trajectory_age=%.3f",
      freshness.reason.c_str(), freshness.ages.odom_age_sec,
      freshness.ages.trajectory_age_sec);
  publishStaleDebug(stamp, freshness);
  resetSteeringLimiter();
  if (stop_on_stale_input_ ||
      state_lattice_v4_poc_command_activation_enabled_) {
    publishStopForStaleInput(stamp, freshness);
  }
  return true;
}

bool SimplePurePursuit::v4PocCommandContractReady(double now_sec) const {
  if (!last_valid_override_contract_received_ ||
      overtake_override_contract_kind_ !=
          OvertakeOverrideContractKind::SPATIAL_LATERAL_AND_SPEED_V4 ||
      !overtake_lateral_override_active_ ||
      overtake_longitudinal_offsets_m_.empty() ||
      !overtakeOverrideFresh(now_sec) || trajectory_ == nullptr) {
    return false;
  }

  if (!state_lattice_v4_poc_identity_gate_enabled_) {
    return true;
  }
  if (!state_lattice_v4_poc_identity_.has_value()) {
    return false;
  }

  // Exact command activation consumes the immutable Cartesian proposal
  // accepted by BindingStore. Its source/base identity is already bound and
  // lease-checked there. A newer baseline callback must not invalidate that
  // still-current proposal by comparing it against a different trajectory.
  if (state_lattice_v4_poc_command_activation_enabled_) {
    if (!state_lattice_v2_control_trajectory_cache_.has_value()) {
      return false;
    }
    const auto &cache = state_lattice_v2_control_trajectory_cache_.value();
    return cache.trajectory != nullptr && cache.proposal != nullptr &&
           cache.identity.plan_generation == overtake_override_generation_ &&
           stateLatticeV2IdentityMatches(
               cache.identity, state_lattice_v4_poc_identity_.value()) &&
           stateLatticeV2IdentityMatches(cache.identity,
                                         cache.proposal->identity);
  }

  return v4PocIdentityMatches(
      state_lattice_v4_poc_identity_.value(), overtake_override_generation_,
      trajectory_->header.frame_id, trajectory_->header.stamp,
      reference_source_generation_);
}

bool SimplePurePursuit::v4PocCommandContractAvailable(double now_sec) const {
  if (v4PocCommandContractReady(now_sec)) {
    return true;
  }
  if (!use_overtake_reference_override_ ||
      !last_valid_override_contract_received_) {
    return false;
  }
  if (!last_valid_override_contract_inactive_) {
    return overtake_override_contract_kind_ ==
               OvertakeOverrideContractKind::SPEED_ONLY_V2 &&
           hasLatchedSpeedOnlyCap() && overtakeOverrideFresh(now_sec);
  }
  const double age_sec =
      inputAgeSec(last_valid_override_contract_sec_, now_sec);
  return ageFresh(age_sec, overtake_override_timeout_sec_) &&
         ageFresh(age_sec, max_override_age_sec_);
}

void SimplePurePursuit::clearStaleOvertakeOverride(double now_sec) {
  if (!overtake_override_active_ || overtakeOverrideFresh(now_sec)) {
    return;
  }

  const double override_age_sec =
      inputAgeSec(last_overtake_override_sec_, now_sec);
  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(),
      static_cast<int>(std::max(0.1, diagnostic_throttle_sec_) * 1000.0),
      "PurePursuit overtake override stale: age=%.3f", override_age_sec);
  if (hasLatchedSpeedOnlyCap()) {
    return;
  }
  clearOvertakeOverride();
  last_valid_override_contract_received_ = false;
}

SimplePurePursuit::ControlTrajectoryContext
SimplePurePursuit::selectControlTrajectory(
    const ControlPosePrediction &control_pose, double now_sec,
    const HorizonFreshnessResult &mpc_horizon_freshness) {
  ControlTrajectoryContext context;
  context.mpc_horizon_freshness = mpc_horizon_freshness;
  if (state_lattice_v4_poc_command_activation_enabled_ &&
      (state_lattice_v2_command_activation_enabled_ ||
       !v4PocCommandContractAvailable(now_sec))) {
    context.invalid_reason = "state_lattice_v4_unavailable";
    return context;
  }
  if (state_lattice_v2_command_activation_enabled_) {
    const auto &cache = state_lattice_v2_control_trajectory_cache_.value();
    context.owned_trajectory = cache.trajectory;
    context.selected_cartesian_proposal = cache.proposal;
    context.base_trajectory = context.owned_trajectory.get();
    context.trajectory = context.owned_trajectory.get();
    context.source = "state_lattice_v2";
    if (context.trajectory == nullptr || context.trajectory->points.empty()) {
      context.invalid_reason = "state_lattice_v2_empty_trajectory";
      return context;
    }
    context.nearest_index =
        findNearestIndex(context.trajectory->points, control_pose.position);
    context.base_nearest_index = context.nearest_index;
    context.valid = true;
    return context;
  }
  const bool v4_cartesian_required =
      state_lattice_v4_poc_command_activation_enabled_ &&
      state_lattice_v4_poc_identity_gate_enabled_ &&
      overtake_override_contract_kind_ ==
          OvertakeOverrideContractKind::SPATIAL_LATERAL_AND_SPEED_V4 &&
      overtake_lateral_override_active_;
  if (v4_cartesian_required) {
    if (!state_lattice_v4_poc_identity_gate_enabled_ ||
        !state_lattice_v4_poc_identity_.has_value() ||
        !state_lattice_v2_control_trajectory_cache_.has_value() ||
        state_lattice_v2_control_trajectory_cache_->trajectory == nullptr ||
        !stateLatticeV2IdentityMatches(
            state_lattice_v2_control_trajectory_cache_->identity,
            state_lattice_v4_poc_identity_.value())) {
      context.invalid_reason = "state_lattice_v4_cartesian_unavailable";
      return context;
    }
    const auto &cache = state_lattice_v2_control_trajectory_cache_.value();
    context.owned_trajectory = cache.trajectory;
    context.selected_cartesian_proposal = cache.proposal;
    context.base_trajectory = trajectory_.get();
    context.trajectory = context.owned_trajectory.get();
    context.source = "state_lattice_v4_cartesian";
    context.v4_poc_contract = true;
    context.v4_poc_identity_required = true;
    context.v4_poc_identity_matched = true;
    context.v4_poc_geometry_applied = true;
    context.v4_poc_generation = overtake_override_generation_;
    context.overtake_override_applied = true;
    context.overtake_override_apply_reason = "typed_cartesian_exact";
    context.overtake_spatial_horizon_required_arc_m =
        lookahead_min_distance_;
    if (context.trajectory == nullptr || context.trajectory->points.empty() ||
        context.base_trajectory == nullptr ||
        context.base_trajectory->points.empty()) {
      context.invalid_reason = "state_lattice_v4_cartesian_empty";
      return context;
    }
    context.nearest_index =
        findNearestIndex(context.trajectory->points, control_pose.position);
    context.base_nearest_index = findNearestIndex(
        context.base_trajectory->points, control_pose.position);
    context.valid = true;
    return context;
  }
  context.mpc_horizon_applied = mpc_horizon_freshness.usable;
  context.trajectory = context.mpc_horizon_applied
                           ? mpc_predicted_horizon_.get()
                           : trajectory_.get();
  // Base means the exact pre-adaptation source selected this cycle. For an
  // MPC cycle it is the horizon itself, not the unrelated reference path.
  context.base_trajectory = context.trajectory;
  context.source = context.mpc_horizon_applied ? "mpc_horizon" : "trajectory";
  if (mpc_predicted_horizon_ != nullptr) {
    context.evaluated_horizon_stamp_sec =
        mpc_predicted_horizon_->header.stamp.sec;
    context.evaluated_horizon_stamp_nanosec =
        mpc_predicted_horizon_->header.stamp.nanosec;
  }
  if (context.mpc_horizon_applied && mpc_predicted_horizon_ != nullptr) {
    context.applied_horizon_stamp_sec = context.evaluated_horizon_stamp_sec;
    context.applied_horizon_stamp_nanosec =
        context.evaluated_horizon_stamp_nanosec;
    if (mpc_horizon_contract_received_ &&
        context.applied_horizon_stamp_sec == mpc_horizon_contract_stamp_sec_ &&
        context.applied_horizon_stamp_nanosec ==
            mpc_horizon_contract_stamp_nanosec_) {
      context.applied_horizon_source = mpc_horizon_contract_source_;
      context.applied_horizon_mode_id = mpc_horizon_contract_mode_id_;
      context.applied_horizon_generation = mpc_horizon_contract_generation_;
    }
  }

  if (context.trajectory == nullptr || context.trajectory->points.empty()) {
    context.invalid_reason =
        context.mpc_horizon_applied ? "empty_mpc_horizon" : "empty_trajectory";
    return context;
  }

  context.nearest_index =
      findNearestIndex(context.trajectory->points, control_pose.position);
  context.base_nearest_index = context.nearest_index;
  context.v4_poc_contract =
      overtake_override_contract_kind_ ==
      OvertakeOverrideContractKind::SPATIAL_LATERAL_AND_SPEED_V4;
  context.v4_poc_identity_required =
      context.v4_poc_contract && state_lattice_v4_poc_identity_gate_enabled_;
  context.v4_poc_generation =
      context.v4_poc_contract ? overtake_override_generation_ : 0U;
  context.v4_poc_identity_matched =
      context.v4_poc_identity_required &&
      state_lattice_v4_poc_identity_.has_value() &&
      v4PocIdentityMatches(
          state_lattice_v4_poc_identity_.value(), overtake_override_generation_,
          context.trajectory->header.frame_id, context.trajectory->header.stamp,
          reference_source_generation_);

  // MPC horizonを使っている周期は、horizon自体を優先する。
  // 通常trajectory周期だけ、plannerから来るovertake overrideを重ねる。
  if (!context.mpc_horizon_applied && overtakeOverrideFresh(now_sec)) {
    context.owned_trajectory = std::make_shared<Trajectory>(*trajectory_);
    context.overtake_override_applied =
        applyOvertakeOverride(*context.owned_trajectory, context.nearest_index,
                              now_sec, &context.overtake_spatial_horizon_arc_m,
                              &context.overtake_spatial_horizon_required_arc_m,
                              &context.overtake_override_apply_reason);
    context.v4_poc_geometry_applied =
        context.v4_poc_contract &&
        (!context.v4_poc_identity_required ||
         context.v4_poc_identity_matched) &&
        context.overtake_override_applied;
    if (context.overtake_override_applied) {
      context.trajectory = context.owned_trajectory.get();
      context.source = "trajectory_overtake_override";
      context.nearest_index =
          findNearestIndex(context.trajectory->points, control_pose.position);
    }
  } else if (!context.mpc_horizon_applied && overtake_override_active_ &&
             !hasLatchedSpeedOnlyCap()) {
    clearOvertakeOverride();
  }

  if (state_lattice_v4_poc_command_activation_enabled_ &&
      context.v4_poc_contract &&
      (!context.v4_poc_geometry_applied ||
       context.source != "trajectory_overtake_override")) {
    context.invalid_reason = "state_lattice_v4_unavailable";
    return context;
  }

  context.valid = true;
  return context;
}

SimplePurePursuit::LongitudinalCommand
SimplePurePursuit::computeLongitudinalCommand(
    const ControlTrajectoryContext &context, double now_sec) const {
  LongitudinalCommand result;
  const auto &nearest = context.trajectory->points.at(context.nearest_index);

  result.target_speed_mps = use_external_target_vel_
                                ? external_target_vel_
                                : nearest.longitudinal_velocity_mps;
  // The reference trajectory is the execution profile authority.  An
  // external target may request a lower speed, but must never raise the raw PP
  // target above the profile carried by the same trajectory generation.
  result.target_speed_mps = applyExecutionProfileSpeedCap(
      result.target_speed_mps,
      static_cast<double>(nearest.longitudinal_velocity_mps));
  result.current_speed_mps = odometry_->twist.twist.linear.x;

  // neutral horizonの先頭点は、現在姿勢アンカーとして低速/0mpsに
  // なることがあるため、少し前方の速度capを見る。
  const bool skip_mpc_horizon_anchor_speed =
      mpc_predicted_horizon_source_ == "neutral_reference" ||
      mpc_predicted_horizon_source_ == "fixed_neutral_reference";
  const auto cap_index = context.mpc_horizon_applied
                             ? selectMpcHorizonVelocityCapIndex(
                                   *context.trajectory, context.nearest_index,
                                   kMpcHorizonVelocityCapForwardArcM,
                                   skip_mpc_horizon_anchor_speed)
                             : context.nearest_index;
  result.speed_cap_trajectory_index = cap_index;
  result.mpc_horizon_velocity_cap_mps =
      context.mpc_horizon_applied
          ? context.trajectory->points.at(cap_index).longitudinal_velocity_mps
          : -1.0;
  if (context.mpc_horizon_applied &&
      std::isfinite(result.mpc_horizon_velocity_cap_mps) &&
      result.mpc_horizon_velocity_cap_mps >= 0.0) {
    result.mpc_horizon_velocity_cap_applied =
        result.mpc_horizon_velocity_cap_mps < result.target_speed_mps;
    result.target_speed_mps =
        std::min(result.target_speed_mps, result.mpc_horizon_velocity_cap_mps);
  }

  if (const auto overtake_speed_cap = overtakeSpeedCap(0, now_sec)) {
    result.overtake_speed_cap_mps = overtake_speed_cap.value();
    result.target_speed_mps =
        applyOvertakeSpeedCap(result.target_speed_mps, overtake_speed_cap);
  }

  result.acceleration_mps2 = proportionalLongitudinalAcceleration(
      result.target_speed_mps, result.current_speed_mps,
      speed_proportional_gain_);
  return result;
}

SimplePurePursuit::LateralCommand SimplePurePursuit::computeLateralCommand(
    const ControlTrajectoryContext &context,
    const ControlPosePrediction &control_pose,
    const LongitudinalCommand &longitudinal) {
  LateralCommand result;
  const auto &trajectory = *context.trajectory;

  const LookaheadParams lookahead_params{
      lookahead_gain_, lookahead_min_distance_,
      curvature_adaptive_lookahead_enabled_, curvature_lookahead_min_distance_,
      curvature_lookahead_sensitivity_};
  result.base_lookahead_distance_m = speedBasedLookaheadDistance(
      longitudinal.target_speed_mps, longitudinal.current_speed_mps,
      lookahead_params);

  // 曲率を見る距離は、速度由来のlookaheadを基準にしつつ上限を持たせる。
  // 直線では遠く、コーナーでは手前を見るための前処理。
  const double curvature_window_ratio =
      std::isfinite(curvature_lookahead_window_ratio_)
          ? std::max(1.0, curvature_lookahead_window_ratio_)
          : 1.0;
  const double curvature_min_arc_m =
      std::isfinite(curvature_lookahead_min_arc_length_)
          ? std::max(1.0e-3, curvature_lookahead_min_arc_length_)
          : 1.0e-3;
  const double curvature_window_max_m =
      std::isfinite(curvature_lookahead_max_window_distance_)
          ? std::max(curvature_min_arc_m,
                     curvature_lookahead_max_window_distance_)
          : result.base_lookahead_distance_m;
  result.curvature_window_distance_m =
      std::min(curvature_window_max_m,
               result.base_lookahead_distance_m * curvature_window_ratio);
  std::size_t unsigned_curvature_last_read = context.nearest_index;
  std::size_t signed_curvature_last_read = context.nearest_index;
  result.path_curvature_1pm = estimateTrajectoryCurvature(
      trajectory, context.nearest_index, result.curvature_window_distance_m,
      curvature_min_arc_m, &unsigned_curvature_last_read);
  result.signed_path_curvature_1pm = estimateSignedTrajectoryCurvature(
      trajectory, context.nearest_index, result.curvature_window_distance_m,
      curvature_min_arc_m, &signed_curvature_last_read);
  result.curvature_last_read_trajectory_index =
      std::max(unsigned_curvature_last_read, signed_curvature_last_read);
  result.desired_lookahead_distance_m = adaptiveLookaheadDistance(
      longitudinal.target_speed_mps, longitudinal.current_speed_mps,
      result.path_curvature_1pm, lookahead_params);
  result.lookahead_distance_m = result.desired_lookahead_distance_m;
  if (curvature_adaptive_lookahead_enabled_) {
    result.lookahead_distance_m = smoothLookaheadDistance(
        result.desired_lookahead_distance_m, smoothed_lookahead_distance_,
        has_smoothed_lookahead_distance_, curvature_lookahead_smoothing_alpha_);
    smoothed_lookahead_distance_ = result.lookahead_distance_m;
    has_smoothed_lookahead_distance_ = true;
  } else {
    has_smoothed_lookahead_distance_ = false;
  }

  // PurePursuitは後輪中心からlookahead点を見るため、制御姿勢から後輪中心へ戻す。
  result.rear_x =
      control_pose.position.x - wheel_base_ / 2.0 * std::cos(control_pose.yaw);
  result.rear_y =
      control_pose.position.y - wheel_base_ / 2.0 * std::sin(control_pose.yaw);
  if (context.overtake_override_applied &&
      std::isfinite(context.overtake_spatial_horizon_arc_m) &&
      !trajectory.points.empty()) {
    // 短いが検証済みのprofileでは、未評価終端より先を見ない。探索距離と
    // 操舵式の分母を同じ実chord距離へ揃え、終端fallback時の過小操舵を防ぐ。
    const auto &endpoint = trajectory.points.back().pose.position;
    const double endpoint_chord_m =
        std::hypot(endpoint.x - result.rear_x, endpoint.y - result.rear_y);
    if (std::isfinite(endpoint_chord_m) && endpoint_chord_m > 1.0e-3) {
      result.lookahead_distance_m =
          std::min(result.lookahead_distance_m, endpoint_chord_m);
    }
  }
  auto lookahead_point_itr =
      std::find_if(trajectory.points.begin() + context.nearest_index,
                   trajectory.points.end(), [&](const TrajectoryPoint &point) {
                     return std::hypot(point.pose.position.x - result.rear_x,
                                       point.pose.position.y - result.rear_y) >=
                            result.lookahead_distance_m;
                   });
  if (lookahead_point_itr == trajectory.points.end()) {
    lookahead_point_itr = std::prev(trajectory.points.end());
    result.lookahead_endpoint_fallback = true;
  }
  result.lookahead_selected_trajectory_index = static_cast<std::size_t>(
      std::distance(trajectory.points.begin(), lookahead_point_itr));
  result.lookahead_point_x = lookahead_point_itr->pose.position.x;
  result.lookahead_point_y = lookahead_point_itr->pose.position.y;

  result.alpha_rad =
      normalizeAngle(std::atan2(result.lookahead_point_y - result.rear_y,
                                result.lookahead_point_x - result.rear_x) -
                     control_pose.yaw);
  result.pure_pursuit_steering_tire_angle_rad =
      std::atan2(2.0 * wheel_base_ * std::sin(result.alpha_rad),
                 result.lookahead_distance_m);
  result.curvature_feedforward_steering_rad =
      std::clamp(horizon_curvature_feedforward_gain_ *
                     std::atan(wheel_base_ * result.signed_path_curvature_1pm),
                 -std::max(0.0, horizon_curvature_feedforward_max_rad_),
                 std::max(0.0, horizon_curvature_feedforward_max_rad_));
  result.raw_steering_tire_angle_rad =
      result.pure_pursuit_steering_tire_angle_rad +
      result.curvature_feedforward_steering_rad;
  result.requested_output_steering_tire_angle_rad =
      steering_tire_angle_gain_ * result.raw_steering_tire_angle_rad;
  // Seed from a fresh measured angle at startup/source changes, then advance
  // from the previous command that was actually published. Re-anchoring every
  // cycle to a lagging steering report can trap the command near zero.
  const bool same_limiter_source = steering_limiter_initialized_ &&
                                   steering_limiter_source_ == context.source;
  const bool fresh_measured_steering =
      control_pose.steering_source == "steering_status" &&
      std::isfinite(control_pose.current_steering_rad);
  const double limiter_reference_rad =
      same_limiter_source
          ? last_commanded_steering_tire_angle_
          : (fresh_measured_steering ? control_pose.current_steering_rad : 0.0);
  result.limiter_reference_steering_rad = limiter_reference_rad;
  result.limiter_reference_valid = std::isfinite(limiter_reference_rad);
  const auto bounded = boundSteeringCommand(
      result.requested_output_steering_tire_angle_rad, limiter_reference_rad,
      steering_command_nominal_dt_sec_,
      free_run_live_exact_hard_steering_limit_rad_,
      free_run_live_exact_hard_steering_rate_limit_radps_);
  result.steering_limits_valid = bounded.valid;
  result.steering_angle_limited = bounded.angle_limited;
  result.steering_rate_limited = bounded.rate_limited;
  result.measured_steering_rad = control_pose.current_steering_rad;
  result.measured_steering_age_sec = control_pose.steering_age_sec;
  result.measured_steering_fresh = fresh_measured_steering;
  result.requested_steering_tire_rotation_rate_radps =
      bounded.requested_rate_radps;
  result.steering_tire_rotation_rate_radps = bounded.bounded_rate_radps;
  result.steering_tire_angle_rad =
      bounded.valid ? bounded.bounded_angle_rad : 0.0;
  return result;
}

void SimplePurePursuit::commitSteeringLimiterCommand(
    double steering_rad, const std::string &source) {
  last_commanded_steering_tire_angle_ = steering_rad;
  steering_limiter_initialized_ = true;
  steering_limiter_source_ = source;
}

void SimplePurePursuit::resetSteeringLimiter() {
  last_commanded_steering_tire_angle_ = 0.0;
  steering_limiter_initialized_ = false;
  steering_limiter_source_.clear();
}

void SimplePurePursuit::publishLookaheadPoint(double x, double y, double z) {
  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point.x = x;
  lookahead_point_msg.point.y = y;
  lookahead_point_msg.point.z = z;
  pub_lookahead_point_->publish(lookahead_point_msg);
}

double SimplePurePursuit::steadyNowSec() const {
  using clock = std::chrono::steady_clock;
  const auto now = clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

SimplePurePursuit::ControlPosePrediction
SimplePurePursuit::predictControlPose(double now_sec) const {
  ControlPosePrediction result;
  if (!odometry_) {
    result.steering_source = "missing_odom";
    return result;
  }

  result.position = odometry_->pose.pose.position;
  result.yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  result.velocity_mps = odometry_->twist.twist.linear.x;

  double steering_age_sec = -1.0;
  const auto [current_steering_rad, steering_source] =
      estimateCurrentSteering(now_sec, &steering_age_sec);
  result.current_steering_rad = current_steering_rad;
  result.applied_steering_rad = current_steering_rad;
  result.steering_age_sec = steering_age_sec;
  result.steering_source = steering_source;

  const bool delay_enabled =
      std::isfinite(pp_control_delay_sec_) && pp_control_delay_sec_ > 0.0;
  const bool steering_fresh = steering_source == "steering_status";
  const double min_compensation_velocity_mps =
      std::max(0.0, min_velocity_for_delay_compensation_mps_);
  const bool velocity_valid =
      std::isfinite(result.velocity_mps) &&
      result.velocity_mps >= min_compensation_velocity_mps;
  if (!delay_enabled || !steering_fresh || !velocity_valid) {
    return result;
  }

  const EgoControlState state{result.position.x, result.position.y, result.yaw,
                              result.velocity_mps};
  const auto predicted = predictDelayedPose(
      state, current_steering_rad, last_commanded_steering_tire_angle_,
      pp_control_delay_sec_, pp_prediction_dt_sec_, steering_time_constant_sec_,
      wheel_base_);
  result.position.x = predicted.x;
  result.position.y = predicted.y;
  result.yaw = predicted.yaw;
  result.applied_steering_rad = predicted.applied_steering_rad;
  result.prediction_steps = predicted.prediction_steps;
  result.shifted = predicted.shifted;
  return result;
}

std::pair<double, std::string>
SimplePurePursuit::estimateCurrentSteering(double now_sec,
                                           double *steering_age_sec) const {
  if (last_steering_status_receive_sec_.has_value()) {
    const double age_sec =
        inputAgeSec(last_steering_status_receive_sec_, now_sec);
    if (steering_age_sec != nullptr) {
      *steering_age_sec = age_sec;
    }
    if (ageFresh(age_sec, steering_status_timeout_sec_) &&
        std::isfinite(latest_steering_status_rad_)) {
      return {latest_steering_status_rad_, "steering_status"};
    }
    return {last_commanded_steering_tire_angle_, "stale_steering_status"};
  }

  if (steering_age_sec != nullptr) {
    *steering_age_sec = -1.0;
  }
  return {last_commanded_steering_tire_angle_, "missing_steering_status"};
}

FreshnessResult
SimplePurePursuit::evaluateInputFreshness(double now_sec) const {
  FreshnessResult result;
  result.ages.odom_age_sec = inputAgeSec(last_odometry_receive_sec_, now_sec);
  result.ages.trajectory_age_sec =
      inputAgeSec(last_trajectory_receive_sec_, now_sec);
  result.ages.override_age_sec =
      overtake_override_active_
          ? inputAgeSec(last_overtake_override_sec_, now_sec)
          : -1.0;

  if (!last_odometry_receive_sec_.has_value() || !odometry_) {
    result.reason = "missing_odom";
    return result;
  }
  if (!ageFresh(result.ages.odom_age_sec, max_odom_age_sec_)) {
    result.reason = "stale_odom";
    return result;
  }
  if (!odometryControlValuesValid(*odometry_)) {
    result.reason = "nonfinite_odom";
    return result;
  }

  const bool trajectory_time_fresh =
      last_trajectory_receive_sec_.has_value() &&
      ageFresh(result.ages.trajectory_age_sec, max_trajectory_age_sec_);
  if (trajectory_ && !trajectory_->points.empty() && trajectory_time_fresh) {
    result.fresh = true;
    result.reason = "fresh";
    return result;
  }

  const auto horizon = evaluateMpcPredictedHorizon(now_sec);
  if (horizon.usable) {
    result.fresh = true;
    result.reason = "fresh_mpc_horizon";
    return result;
  }

  if (!last_trajectory_receive_sec_.has_value() || !trajectory_) {
    result.reason = "missing_trajectory";
    return result;
  }
  if (trajectory_->points.empty()) {
    result.reason = "empty_trajectory";
    return result;
  }
  if (!trajectory_time_fresh) {
    result.reason = "stale_trajectory";
    return result;
  }
  return result;
}

HorizonFreshnessResult
SimplePurePursuit::evaluateMpcPredictedHorizon(double now_sec) const {
  if (!odometry_) {
    HorizonFreshnessResult result;
    result.reason = "missing_odom";
    result.age_sec =
        inputAgeSec(last_mpc_predicted_horizon_receive_sec_, now_sec);
    return result;
  }

  const std::size_t point_count =
      mpc_predicted_horizon_ ? mpc_predicted_horizon_->points.size() : 0;
  const std::size_t min_points =
      static_cast<std::size_t>(std::max(1, min_mpc_horizon_points_));
  const bool frame_valid = mpc_predicted_horizon_ &&
                           mpc_predicted_horizon_->header.frame_id == "map";
  const bool values_valid =
      mpc_predicted_horizon_ && trajectoryValuesValid(*mpc_predicted_horizon_);
  const double start_distance_m =
      mpc_predicted_horizon_
          ? firstPointDistanceToEgo(*mpc_predicted_horizon_, *odometry_)
          : -1.0;
  const double arc_length_m = mpc_predicted_horizon_
                                  ? trajectoryArcLength(*mpc_predicted_horizon_)
                                  : -1.0;

  auto result = evaluateMpcHorizonFreshness(
      use_mpc_predicted_horizon_, last_mpc_predicted_horizon_receive_sec_,
      now_sec, max_mpc_horizon_age_sec_, point_count, min_points,
      start_distance_m, max_mpc_horizon_start_distance_m_, arc_length_m,
      min_mpc_horizon_arc_length_m_, values_valid, frame_valid);

  if (result.usable && require_solved_mpc_health_for_horizon_) {
    const double health_age_sec = mpcHealthAgeSec(now_sec);
    if (!last_mpc_health_receive_sec_.has_value()) {
      result.usable = false;
      result.reason = "mpc_health_missing";
    } else if (!ageFresh(health_age_sec, max_mpc_health_age_sec_)) {
      result.usable = false;
      result.reason = "mpc_health_stale";
    } else if (mpc_health_status_ != "solved" ||
               mpc_health_infeasible_count_ > 0) {
      result.usable = false;
      result.reason = mpc_health_infeasible_count_ > 0
                          ? "mpc_health_infeasible"
                          : "mpc_health_" + mpc_health_status_;
    }
  }

  if (result.usable) {
    const bool override_active = overtakeOverrideFresh(now_sec);
    const bool stamp_matches = mpc_predicted_horizon_ &&
                               mpc_horizon_contract_received_ &&
                               mpc_predicted_horizon_->header.stamp.sec ==
                                   mpc_horizon_contract_stamp_sec_ &&
                               mpc_predicted_horizon_->header.stamp.nanosec ==
                                   mpc_horizon_contract_stamp_nanosec_;
    const auto contract = evaluateMpcHorizonContract(
        require_matching_overtake_horizon_contract_, override_active,
        overtake_mode_id_, overtake_override_generation_,
        mpc_horizon_contract_received_,
        last_mpc_predicted_horizon_contract_receive_sec_, now_sec,
        max_mpc_horizon_age_sec_, stamp_matches, mpc_horizon_contract_source_,
        mpc_horizon_contract_mode_id_, mpc_horizon_contract_generation_,
        overtake_solver_horizon_authorized_,
        mpc_horizon_contract_solver_horizon_authorized_,
        overtake_mandatory_lateral_avoidance_,
        mpc_horizon_contract_mandatory_lateral_avoidance_);
    if (!contract.usable) {
      result.usable = false;
      result.reason = contract.reason;
    }
  }

  if (result.usable) {
    const auto nearest_idx = findNearestIndex(mpc_predicted_horizon_->points,
                                              odometry_->pose.pose.position);
    if (nearest_idx > kMaxMpcHorizonNearestIndex) {
      result.usable = false;
      result.reason = "nearest_index";
    }
  }

  return result;
}

double SimplePurePursuit::mpcHealthAgeSec(double now_sec) const {
  return inputAgeSec(last_mpc_health_receive_sec_, now_sec);
}

void SimplePurePursuit::publishStateLatticeV2BaseAttestation(
    const ControllerCommandEnvelope &command_envelope,
    const ControlTrajectoryContext &context) {
  using BindingStatus = StateLatticeV2BindingStatus;
  if (state_lattice_v2_base_attestation_attempt_count_ !=
      std::numeric_limits<std::uint64_t>::max()) {
    ++state_lattice_v2_base_attestation_attempt_count_;
  }
  state_lattice_v2_base_attestation_build_result_ =
      BindingStatus::BASE_ATTESTATION_RESULT_NOT_RUN;
  state_lattice_v2_base_attestation_build_diagnostic_ =
      BindingStatus::BASE_ATTESTATION_BUILD_DIAGNOSTIC_NOT_RUN;
  state_lattice_v2_base_attestation_validation_reason_ =
      BindingStatus::BASE_ATTESTATION_VALIDATION_REASON_NOT_RUN;
  state_lattice_v2_base_attestation_point_invalid_field_ =
      BindingStatus::BASE_ATTESTATION_POINT_FIELD_NOT_RUN;
  state_lattice_v2_base_attestation_failure_window_index_ =
      BindingStatus::BASE_ATTESTATION_NO_POINT_INDEX;
  state_lattice_v2_base_attestation_failure_source_index_ =
      BindingStatus::BASE_ATTESTATION_NO_POINT_INDEX;
  state_lattice_v2_base_attestation_snapshot_result_ =
      BindingStatus::BASE_ATTESTATION_RESULT_NOT_RUN;
  state_lattice_v2_base_attestation_local_reject_reason_ =
      BindingStatus::BASE_ATTESTATION_RESULT_NOT_RUN;
  state_lattice_v2_base_attestation_race_arm_epoch_ =
      state_lattice_shadow_race_arm_epoch_.epoch();
  if (!state_lattice_v2_base_attestation_publish_enabled_) {
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_DISABLED;
    return;
  }
  if (pub_state_lattice_v2_base_attestation_ == nullptr) {
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_PUBLISHER_MISSING;
    return;
  }
  if (state_lattice_v2_binding_store_ == nullptr) {
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_BINDING_STORE_MISSING;
    return;
  }
  if (context.base_trajectory == nullptr) {
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_BASE_TRAJECTORY_MISSING;
    return;
  }
  if (!state_lattice_shadow_race_arm_epoch_.armed()) {
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_RACE_NOT_ARMED;
    return;
  }
  if (state_lattice_shadow_race_arm_epoch_.epoch() == 0U) {
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_RACE_EPOCH_ZERO;
    return;
  }
  if (state_lattice_v2_base_attestation_session_id_ !=
      std::to_string(state_lattice_shadow_race_arm_epoch_.epoch())) {
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_SESSION_EPOCH_MISMATCH;
    return;
  }

  const bool mpc_source = context.mpc_horizon_applied;
  const std::uint32_t source_generation =
      mpc_source && context.applied_horizon_generation > 0U
          ? context.applied_horizon_generation
          : (mpc_source ? mpc_source_generation_
                        : reference_source_generation_);
  const double lease_duration_sec =
      mpc_source ? max_mpc_horizon_age_sec_ : max_trajectory_age_sec_;
  std::uint64_t base_lease_id =
      producer_instance_id_ + command_envelope.command_sequence;
  if (base_lease_id == 0U) {
    base_lease_id = command_envelope.command_sequence;
  }

  Ay0BaseCaptureInput input;
  input.trajectory = context.base_trajectory;
  input.nearest_source_index = context.base_nearest_index;
  input.source_kind =
      mpc_source
          ? overtake_transport_contract::c002ay0::kFixedBaseSourceMpcHorizon
          : overtake_transport_contract::c002ay0::
                kFixedBaseSourceReferenceTrajectory;
  input.source_generation = source_generation;
  input.record_stamp = toAy0FixedTime(command_envelope.header.stamp);
  input.lease_valid_until =
      addSeconds(context.base_trajectory->header.stamp, lease_duration_sec);
  // These fixed-record transport fields are required by the existing
  // canonical builder but are not used as live identity.  The direct message
  // carries the explicit bounded producer/session IDs below.
  input.session_generation =
      c002ay0_session_generation_ != 0U ? c002ay0_session_generation_ : 1U;
  input.session_nonce =
      c002ay0_session_nonce_ != 0U ? c002ay0_session_nonce_ : 1U;
  input.race_arm_epoch = state_lattice_shadow_race_arm_epoch_.epoch();
  input.controller_instance_id = producer_instance_id_;
  input.controller_sequence = command_envelope.command_sequence;
  input.base_lease_id = base_lease_id;
  input.controller_implementation_sha256 =
      c002ay0_controller_implementation_digest_;
  input.controller_config_sha256 = c002ay0_controller_config_digest_;

  StateLatticeV2BaseAttestation attestation;
  Ay0BaseCaptureDiagnosticStage build_diagnostic;
  Ay0BaseCaptureValidationReason validation_reason;
  Ay0BaseCapturePointDiagnostic point_diagnostic;
  const auto build_result =
      buildAy0BaseSnapshot(input, attestation.base, build_diagnostic,
                           validation_reason, point_diagnostic);
  state_lattice_v2_base_attestation_build_result_ =
      static_cast<std::uint8_t>(build_result);
  state_lattice_v2_base_attestation_build_diagnostic_ =
      static_cast<std::uint8_t>(build_diagnostic);
  state_lattice_v2_base_attestation_validation_reason_ =
      static_cast<std::uint8_t>(validation_reason);
  state_lattice_v2_base_attestation_point_invalid_field_ =
      static_cast<std::uint8_t>(point_diagnostic.field);
  state_lattice_v2_base_attestation_failure_window_index_ =
      point_diagnostic.window_index;
  state_lattice_v2_base_attestation_failure_source_index_ =
      point_diagnostic.source_index;
  if (build_result != Ay0BaseCaptureResult::kBuilt) {
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_FIXED_RECORD_REJECTED;
    return;
  }
  const auto snapshot_result =
      overtake_transport_contract::c002ay0::validateBaseSnapshotV1(
          attestation.base);
  state_lattice_v2_base_attestation_snapshot_result_ =
      static_cast<std::uint8_t>(snapshot_result);
  if (snapshot_result !=
      overtake_transport_contract::c002ay0::ValidationError::NONE) {
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_SNAPSHOT_REJECTED;
    return;
  }
  attestation.schema_version =
      StateLatticeV2BaseAttestation::SCHEMA_V1_NON_AUTHORITATIVE;
  attestation.header.stamp = attestation.base.record_stamp;
  attestation.header.frame_id = attestation.base.frame_id;
  attestation.producer_instance_id =
      state_lattice_v2_base_attestation_producer_instance_id_;
  attestation.session_id = state_lattice_v2_base_attestation_session_id_;
  attestation.attestation_sequence = command_envelope.command_sequence;

  const builtin_interfaces::msg::Time now_ros = get_clock()->now();
  const auto candidate_validation =
      overtake_transport_contract::state_lattice_v2::validateBaseAttestation(
          attestation,
          state_lattice_v2_base_attestation_producer_instance_id_,
          state_lattice_v2_base_attestation_session_id_, now_ros);
  if (candidate_validation !=
      overtake_transport_contract::state_lattice_v2::RejectReason::kNone) {
    state_lattice_v2_base_attestation_local_reject_reason_ =
        static_cast<std::uint8_t>(candidate_validation);
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_LOCAL_RECORD_REJECTED;
    return;
  }

  const bool has_previous =
      state_lattice_v2_last_published_base_attestation_.has_value();
  StateLatticeV2BaseAttestation attestation_to_record = attestation;
  if (state_lattice_v2_base_attestation_refresh_pending_) {
    if (!has_previous) {
      state_lattice_v2_base_attestation_local_reject_reason_ =
          static_cast<std::uint8_t>(
              overtake_transport_contract::state_lattice_v2::RejectReason::
                  kBaseAttestationMissing);
      state_lattice_v2_base_attestation_stage_ =
          BindingStatus::BASE_ATTESTATION_LOCAL_RECORD_REJECTED;
      return;
    }
    // The accepted proposal is bound to the previously published exact base.
    // A newer reference may arrive between its callback and this timer. Build
    // the one-shot ratchet from that immutable accepted base, not from the
    // newly selected reference, so the new reference cannot deadlock pending.
    // Only volatile record/command identity changes; the source-derived lease
    // and every semantic field remain byte-identical.
    attestation_to_record =
        state_lattice_v2_last_published_base_attestation_.value();
    attestation_to_record.header.stamp = attestation.header.stamp;
    attestation_to_record.attestation_sequence =
        attestation.attestation_sequence;
    attestation_to_record.base.record_stamp = attestation.base.record_stamp;
    attestation_to_record.base.controller_sequence =
        attestation.base.controller_sequence;
    attestation_to_record.base.base_lease_id = attestation.base.base_lease_id;
    attestation_to_record.base.snapshot_sha256 =
        overtake_transport_contract::c002ay0::canonicalizeBaseSnapshotV1(
            attestation_to_record.base)
            .sha256;
    const auto ratchet_validation =
        overtake_transport_contract::state_lattice_v2::validateBaseAttestation(
            attestation_to_record,
            state_lattice_v2_base_attestation_producer_instance_id_,
            state_lattice_v2_base_attestation_session_id_, now_ros);
    if (ratchet_validation !=
            overtake_transport_contract::state_lattice_v2::RejectReason::kNone ||
        !stateLatticeV2BaseAttestationReusable(
            state_lattice_v2_last_published_base_attestation_.value(),
            attestation_to_record, now_ros)) {
      state_lattice_v2_base_attestation_local_reject_reason_ =
          static_cast<std::uint8_t>(
              ratchet_validation != overtake_transport_contract::
                                        state_lattice_v2::RejectReason::kNone
                  ? ratchet_validation
                  : overtake_transport_contract::state_lattice_v2::
                        RejectReason::kBaseAttestationMismatch);
      state_lattice_v2_base_attestation_stage_ =
          BindingStatus::BASE_ATTESTATION_LOCAL_RECORD_REJECTED;
      return;
    }
  }
  const bool semantic_match =
      has_previous && stateLatticeV2BaseAttestationReusable(
                          state_lattice_v2_last_published_base_attestation_.value(),
                          attestation_to_record, now_ros);
  const bool reuse_previous =
      !state_lattice_v2_base_attestation_refresh_pending_ && semantic_match;

  // Commit the compact local witness before exposing the matching wire
  // attestation.  A multi-threaded executor can otherwise complete the
  // Planner round-trip before this PP records its own base, causing a false
  // BASE_ATTESTATION_MISSING rejection. Semantic duplicates keep the original
  // compact key and immutable message, so per-command volatile fields cannot
  // evict a still-valid Planner base from the bounded exact history.
  overtake_transport_contract::state_lattice_v2::RejectReason reason{};
  if (!reuse_previous &&
      !state_lattice_v2_binding_store_->recordBaseAttestation(
          attestation_to_record, now_ros, &reason)) {
    state_lattice_v2_base_attestation_local_reject_reason_ =
        static_cast<std::uint8_t>(reason);
    state_lattice_v2_base_attestation_stage_ =
        BindingStatus::BASE_ATTESTATION_LOCAL_RECORD_REJECTED;
    return;
  }
  // Publication remains observation-only and occurs after the existing
  // command/status publications at the call site.
  const auto &published_attestation =
      reuse_previous
          ? state_lattice_v2_last_published_base_attestation_.value()
          : attestation_to_record;
  pub_state_lattice_v2_base_attestation_->publish(published_attestation);
  if (!reuse_previous) {
    state_lattice_v2_last_published_base_attestation_ = attestation_to_record;
    // Clear only after record-before-publish completed. A record rejection or
    // publish exception retains pending and cannot silently self-renew.
    state_lattice_v2_base_attestation_refresh_pending_ = false;
  }
  if (state_lattice_v2_base_attestation_publish_count_ !=
      std::numeric_limits<std::uint64_t>::max()) {
    ++state_lattice_v2_base_attestation_publish_count_;
  }
  state_lattice_v2_base_attestation_local_reject_reason_ =
      static_cast<std::uint8_t>(
          overtake_transport_contract::state_lattice_v2::RejectReason::kNone);
  state_lattice_v2_base_attestation_stage_ =
      BindingStatus::BASE_ATTESTATION_PUBLISHED;
}

void SimplePurePursuit::captureAy0BaseShadow(
    const ControllerCommandEnvelope &command_envelope,
    const ControlTrajectoryContext &context) noexcept {
  if (!c002ay0_shadow_capture_enabled_ || c002ay0_worker_session_ == nullptr ||
      context.base_trajectory == nullptr ||
      !state_lattice_shadow_race_arm_epoch_.armed() ||
      state_lattice_shadow_race_arm_epoch_.epoch() == 0U) {
    return;
  }

  const bool mpc_source = context.mpc_horizon_applied;
  const std::uint32_t source_generation =
      mpc_source && context.applied_horizon_generation > 0U
          ? context.applied_horizon_generation
          : (mpc_source ? mpc_source_generation_
                        : reference_source_generation_);
  const double lease_duration_sec =
      mpc_source ? max_mpc_horizon_age_sec_ : max_trajectory_age_sec_;
  std::uint64_t base_lease_id =
      producer_instance_id_ + command_envelope.command_sequence;
  if (base_lease_id == 0U) {
    base_lease_id = command_envelope.command_sequence;
  }

  Ay0BaseCaptureInput input;
  input.trajectory = context.base_trajectory;
  input.nearest_source_index = context.base_nearest_index;
  input.source_kind =
      mpc_source
          ? overtake_transport_contract::c002ay0::kFixedBaseSourceMpcHorizon
          : overtake_transport_contract::c002ay0::
                kFixedBaseSourceReferenceTrajectory;
  input.source_generation = source_generation;
  input.record_stamp = toAy0FixedTime(command_envelope.header.stamp);
  input.lease_valid_until =
      addSeconds(context.base_trajectory->header.stamp, lease_duration_sec);
  input.session_generation = c002ay0_session_generation_;
  input.session_nonce = c002ay0_session_nonce_;
  input.race_arm_epoch = state_lattice_shadow_race_arm_epoch_.epoch();
  input.controller_instance_id = producer_instance_id_;
  input.controller_sequence = command_envelope.command_sequence;
  input.base_lease_id = base_lease_id;
  input.controller_implementation_sha256 =
      c002ay0_controller_implementation_digest_;
  input.controller_config_sha256 = c002ay0_controller_config_digest_;

  overtake_transport_contract::c002ay0::FixedBaseRecord record{};
  if (buildAy0FixedBaseRecord(input, record) != Ay0BaseCaptureResult::kBuilt) {
    return;
  }
  (void)c002ay0_worker_session_->tryCapture(record);
}

void SimplePurePursuit::captureControllerAppliedShadow(
    const ControllerCommandEnvelope &command_envelope,
    const ControlTrajectoryContext *context,
    const ControlPosePrediction *control_pose,
    const AckermannControlCommand &raw_command,
    double required_spatial_horizon_m, std::size_t speed_cap_trajectory_index,
    std::size_t curvature_last_read_trajectory_index,
    std::size_t lookahead_selected_trajectory_index,
    std::size_t required_horizon_end_trajectory_index,
    bool lookahead_endpoint_fallback, double now_sec) noexcept {
  if (aw2_shadow_session_ == nullptr) {
    return;
  }

  aw2_shadow::FixedSnapshot snapshot{};
  auto &key = snapshot.controller_sample_key;
  key.controller_role = aw2_shadow::kControllerRolePrimary;
  key.controller_instance_id = command_envelope.producer_instance_id;
  key.controller_sequence = command_envelope.command_sequence;
  key.controller_command_stamp = toFixedTime(command_envelope.command.stamp);
  key.plan_generation = command_envelope.plan_generation;

  aw2_shadow::ResourceLimitKind limit = aw2_shadow::ResourceLimitKind::kNone;
  if (overtake_plan_ != nullptr) {
    snapshot.typed_plan_present = true;
    snapshot.typed_plan_identity_schema_version =
        overtake_plan_->aw2_identity_schema_version;
    snapshot.typed_plan_trajectory_authorized =
        overtake_plan_->trajectory_authorized;
    snapshot.typed_plan_fresh =
        last_overtake_plan_receive_sec_.has_value() &&
        ageFresh(inputAgeSec(last_overtake_plan_receive_sec_, now_sec),
                 max_override_age_sec_);
    key.race_arm_epoch = overtake_plan_->race_arm_epoch;
    key.planner_instance_id = overtake_plan_->planner_instance_id;
    key.attempt_id = overtake_plan_->attempt_id;
    key.pass_direction = overtake_plan_->pass_direction;
    key.connector_transaction_id = overtake_plan_->connector_transaction_id;
    key.plan_stamp = toFixedTime(overtake_plan_->header.stamp);
    if (!aw2_shadow::copyFixedString(overtake_plan_->target_vehicle_id,
                                     key.target_vehicle_id)) {
      limit = aw2_shadow::ResourceLimitKind::kTarget;
    }
    snapshot.candidate_revision = overtake_plan_->candidate_revision;
    snapshot.candidate_content_sha256 =
        overtake_plan_->candidate_content_sha256;
  }

  snapshot.record_stamp = toFixedTime(command_envelope.header.stamp);
  const std::string_view record_frame =
      context != nullptr && context->trajectory != nullptr
          ? std::string_view(context->trajectory->header.frame_id)
          : std::string_view("base_link");
  if (!aw2_shadow::copyFixedString(record_frame, snapshot.record_frame_id) &&
      limit == aw2_shadow::ResourceLimitKind::kNone) {
    limit = aw2_shadow::ResourceLimitKind::kFrame;
  }

  snapshot.raw_command = toFixedCommand(raw_command);
  snapshot.output_command = toFixedCommand(command_envelope.command);
  snapshot.raw_steering_tire_angle_rad =
      raw_command.lateral.steering_tire_angle;
  snapshot.output_steering_tire_angle_rad =
      command_envelope.command.lateral.steering_tire_angle;
  snapshot.raw_steering_tire_rotation_rate_radps =
      raw_command.lateral.steering_tire_rotation_rate;
  snapshot.output_steering_tire_rotation_rate_radps =
      command_envelope.command.lateral.steering_tire_rotation_rate;
  snapshot.required_spatial_horizon_m = required_spatial_horizon_m;
  snapshot.speed_cap_trajectory_index = static_cast<std::uint32_t>(std::min(
      speed_cap_trajectory_index,
      static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  snapshot.curvature_last_read_trajectory_index =
      static_cast<std::uint32_t>(std::min(
          curvature_last_read_trajectory_index,
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  snapshot.lookahead_selected_trajectory_index =
      static_cast<std::uint32_t>(std::min(
          lookahead_selected_trajectory_index,
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  snapshot.required_horizon_end_trajectory_index =
      static_cast<std::uint32_t>(std::min(
          required_horizon_end_trajectory_index,
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  snapshot.lookahead_endpoint_fallback = lookahead_endpoint_fallback;
  snapshot.rollout_state = ControllerAppliedEnvelope::ROLLOUT_UNAVAILABLE;

  if (context != nullptr && context->trajectory != nullptr) {
    const Trajectory *base = context->base_trajectory != nullptr
                                 ? context->base_trajectory
                                 : context->trajectory;
    const std::size_t first_source_index = context->nearest_index;
    const std::size_t last_source_index =
        std::max({first_source_index, speed_cap_trajectory_index,
                  curvature_last_read_trajectory_index,
                  lookahead_selected_trajectory_index,
                  required_horizon_end_trajectory_index});
    if (required_horizon_end_trajectory_index >=
        context->trajectory->points.size()) {
      limit = aw2_shadow::ResourceLimitKind::kHorizonUnavailable;
    } else if (last_source_index < first_source_index ||
               last_source_index >= context->trajectory->points.size() ||
               last_source_index - first_source_index + 1U >
                   aw2_shadow::kMaxGeometryPoints) {
      limit = aw2_shadow::ResourceLimitKind::kGeometryInterval;
    } else if (!copyFixedGeometryInterval(
                   *context->trajectory, first_source_index, last_source_index,
                   speed_cap_trajectory_index,
                   curvature_last_read_trajectory_index,
                   lookahead_selected_trajectory_index,
                   required_horizon_end_trajectory_index,
                   lookahead_endpoint_fallback, snapshot.applied_geometry) ||
               !copyFixedGeometryInterval(
                   *base, first_source_index, last_source_index,
                   speed_cap_trajectory_index,
                   curvature_last_read_trajectory_index,
                   lookahead_selected_trajectory_index,
                   required_horizon_end_trajectory_index,
                   lookahead_endpoint_fallback, snapshot.base_geometry)) {
      if (limit == aw2_shadow::ResourceLimitKind::kNone) {
        limit = base->header.frame_id.size() > aw2_shadow::kMaxFrameBytes ||
                        context->trajectory->header.frame_id.size() >
                            aw2_shadow::kMaxFrameBytes
                    ? aw2_shadow::ResourceLimitKind::kFrame
                    : aw2_shadow::ResourceLimitKind::kGeometry;
      }
    } else {
      snapshot.nearest_trajectory_index = static_cast<std::uint32_t>(
          std::min<std::size_t>(context->nearest_index,
                                std::numeric_limits<std::uint32_t>::max()));
      double available_m = 0.0;
      for (std::size_t index = first_source_index + 1U;
           index <= last_source_index; ++index) {
        const auto &previous =
            context->trajectory->points[index - 1U].pose.position;
        const auto &current = context->trajectory->points[index].pose.position;
        const double segment_m =
            std::hypot(current.x - previous.x, current.y - previous.y);
        if (!std::isfinite(segment_m) || segment_m < 0.0) {
          limit = aw2_shadow::ResourceLimitKind::kGeometryInterval;
          break;
        }
        available_m += segment_m;
      }
      snapshot.trajectory_progress_m = 0.0;
      snapshot.available_spatial_horizon_m = available_m;
    }
    snapshot.geometry_relation =
        context->overtake_override_applied
            ? ControllerAppliedEnvelope::
                  GEOMETRY_RELATION_DERIVED_REFERENCE_OVERRIDE
            : ControllerAppliedEnvelope::GEOMETRY_RELATION_DIRECT_APPLIED;
    if (context->overtake_override_applied) {
      snapshot.source_generation = overtake_source_generation_;
      snapshot.source_original_size_bytes =
          aw2_overtake_source_wire_.original_size_bytes;
      if (!aw2_overtake_source_wire_.complete ||
          aw2_overtake_source_wire_.bytes.size() >
              aw2_shadow::kMaxSourceBytes) {
        if (limit == aw2_shadow::ResourceLimitKind::kNone) {
          limit = aw2_shadow::ResourceLimitKind::kSource;
        }
      } else {
        snapshot.source_wire_complete = true;
        snapshot.source_wire_size =
            static_cast<std::uint32_t>(aw2_overtake_source_wire_.bytes.size());
        for (std::size_t index = 0U;
             index < aw2_overtake_source_wire_.bytes.size(); ++index) {
          snapshot.source_wire[index] = aw2_overtake_source_wire_.bytes[index];
        }
      }
    }
  }

  if (control_pose != nullptr) {
    snapshot.control_pose.position_x_m = control_pose->position.x;
    snapshot.control_pose.position_y_m = control_pose->position.y;
    snapshot.control_pose.position_z_m = control_pose->position.z;
    snapshot.control_pose.orientation_z = std::sin(control_pose->yaw * 0.5);
    snapshot.control_pose.orientation_w = std::cos(control_pose->yaw * 0.5);
    snapshot.control_pose_stamp = toFixedTime(command_envelope.header.stamp);
  } else {
    snapshot.control_pose.orientation_w = 1.0;
    snapshot.control_pose_stamp = toFixedTime(command_envelope.header.stamp);
  }

  if (limit != aw2_shadow::ResourceLimitKind::kNone) {
    auto resource = aw2_shadow::makeResourceLimitSnapshot(key, limit);
    resource.record_stamp = snapshot.record_stamp;
    if (limit != aw2_shadow::ResourceLimitKind::kFrame) {
      resource.record_frame_id = snapshot.record_frame_id;
    }
    resource.raw_command = snapshot.raw_command;
    resource.output_command = snapshot.output_command;
    resource.candidate_revision = snapshot.candidate_revision;
    resource.candidate_content_sha256 = snapshot.candidate_content_sha256;
    resource.required_spatial_horizon_m = snapshot.required_spatial_horizon_m;
    resource.nearest_trajectory_index = snapshot.nearest_trajectory_index;
    resource.speed_cap_trajectory_index = snapshot.speed_cap_trajectory_index;
    resource.curvature_last_read_trajectory_index =
        snapshot.curvature_last_read_trajectory_index;
    resource.lookahead_selected_trajectory_index =
        snapshot.lookahead_selected_trajectory_index;
    resource.required_horizon_end_trajectory_index =
        snapshot.required_horizon_end_trajectory_index;
    resource.lookahead_endpoint_fallback = snapshot.lookahead_endpoint_fallback;
    (void)aw2_shadow_session_->tryCapture(resource);
    return;
  }
  (void)aw2_shadow_session_->tryCapture(snapshot);
}

void SimplePurePursuit::publishStopForStaleInput(
    const rclcpp::Time &stamp, const FreshnessResult &freshness,
    bool refresh_base_attestation) {
  auto cmd = zeroAckermannControlCommand(stamp);
  cmd.longitudinal.acceleration = -1.5;
  if (recovery_mode_) {
    return;
  }
  ControllerTrackingStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = "base_link";
  status.pp_command_fresh = false;
  status.trajectory_tracking_usable = false;
  status.command_age_sec = std::numeric_limits<float>::infinity();
  status.reason = freshness.reason;
  const auto plan_snapshot = overtake_plan_store_.beginControlCycle();
  status.plan_generation =
      plan_snapshot != nullptr && plan_snapshot->plan != nullptr
          ? plan_snapshot->plan->plan_generation
          : 0U;
  const auto command_envelope = publishControllerCommandEnvelope(
      cmd, status, plan_snapshot.get(), nullptr);
  if (command_envelope.has_value()) {
    publishControllerExecutionEnvelope(command_envelope.value(), nullptr,
                                       nullptr, nullptr, steadyNowSec(),
                                       plan_snapshot.get());
    if (refresh_base_attestation && trajectory_ != nullptr &&
        !trajectory_->points.empty() && odometry_ != nullptr) {
      ControlTrajectoryContext base_context;
      base_context.base_trajectory = trajectory_.get();
      base_context.base_nearest_index =
          findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
      publishStateLatticeV2BaseAttestation(command_envelope.value(),
                                           base_context);
    }
  }
  pub_cmd_->publish(cmd);
  resetSteeringLimiter();
  pub_raw_cmd_->publish(cmd);
  if (command_envelope.has_value()) {
    captureControllerAppliedShadow(command_envelope.value(), nullptr, nullptr,
                                   cmd, 0.0, 0U, 0U, 0U, 0U, false,
                                   steadyNowSec());
  }
}

void SimplePurePursuit::publishStaleDebug(const rclcpp::Time &stamp,
                                          const FreshnessResult &freshness) {
  (void)stamp;
  if (debug_publish_period_sec_ <= 0.0 || !pub_debug_) {
    return;
  }
  const double now_sec = steadyNowSec();
  if (now_sec - last_debug_publish_sec_ < debug_publish_period_sec_) {
    return;
  }
  last_debug_publish_sec_ = now_sec;
  const auto horizon = evaluateMpcPredictedHorizon(now_sec);

  std::ostringstream json;
  json << "{"
       << "\"controller\":\"simple_pure_pursuit\","
       << "\"stale_input\":true,"
       << "\"stale_reason\":\"" << freshness.reason << "\","
       << "\"stop_on_stale_input\":"
       << (stop_on_stale_input_ ? "true" : "false") << ","
       << "\"odom_age_sec\":" << freshness.ages.odom_age_sec << ","
       << "\"trajectory_age_sec\":" << freshness.ages.trajectory_age_sec << ","
       << "\"override_age_sec\":" << freshness.ages.override_age_sec << ","
       << "\"mpc_horizon_age_sec\":" << horizon.age_sec << ","
       << "\"mpc_horizon_points\":" << horizon.point_count << ","
       << "\"mpc_health_required_for_horizon\":"
       << (require_solved_mpc_health_for_horizon_ ? "true" : "false") << ","
       << "\"mpc_health_status\":\"" << mpc_health_status_ << "\","
       << "\"mpc_health_age_sec\":" << mpcHealthAgeSec(now_sec) << ","
       << "\"mpc_infeasible_count\":" << mpc_health_infeasible_count_ << ","
       << "\"mpc_predicted_horizon_source\":\"" << mpc_predicted_horizon_source_
       << "\","
       << "\"mpc_horizon_contract_required\":"
       << (require_matching_overtake_horizon_contract_ ? "true" : "false")
       << ","
       << "\"mpc_horizon_contract_received\":"
       << (mpc_horizon_contract_received_ ? "true" : "false") << ","
       << "\"mpc_horizon_contract_age_sec\":"
       << inputAgeSec(last_mpc_predicted_horizon_contract_receive_sec_, now_sec)
       << ","
       << "\"mpc_horizon_contract_source\":\"" << mpc_horizon_contract_source_
       << "\","
       << "\"mpc_horizon_contract_mode_id\":" << mpc_horizon_contract_mode_id_
       << ","
       << "\"mpc_horizon_contract_generation\":"
       << mpc_horizon_contract_generation_ << ","
       << "\"overtake_override_generation\":" << overtake_override_generation_
       << ","
       << "\"mpc_horizon_reject_reason\":\"" << horizon.reason << "\"}";

  String msg;
  msg.data = json.str();
  pub_debug_->publish(msg);
}

void SimplePurePursuit::onOvertakeOverride(
    const Float32MultiArray::SharedPtr msg) {
  if (!use_overtake_reference_override_) {
    return;
  }

  const double now_sec = steadyNowSec();
  const auto contract = parseOvertakeOverrideContract(msg->data);
  if (!contract.has_value()) {
    pending_state_lattice_v4_contract_.reset();
    if (const auto &retained = overtake_speed_only_latch_.retained();
        retained.has_value()) {
      applyReceivedOvertakeOverride(retained.value());
      last_valid_override_contract_received_ = false;
      return;
    }
    clearOvertakeOverride();
    last_valid_override_contract_received_ = false;
    return;
  }
  if (contract->kind == OvertakeOverrideContractKind::INACTIVE) {
    pending_state_lattice_v4_contract_.reset();
    overtake_source_payload_ = canonicalizeSourcePayload(*msg);
    aw2_overtake_source_wire_ = serializeAw2SourceWire(*msg);
    overtake_source_generation_ = contract->generation;
    clearOvertakeOverride();
    last_overtake_override_sec_ = now_sec;
    last_valid_override_contract_sec_ = now_sec;
    last_valid_override_contract_generation_ = contract->generation;
    last_valid_override_contract_inactive_ = true;
    last_valid_override_contract_received_ = true;
    return;
  }

  auto source_payload = canonicalizeSourcePayload(*msg);
  auto source_wire = serializeAw2SourceWire(*msg);
  if (state_lattice_v4_poc_command_activation_enabled_ &&
      state_lattice_v4_poc_identity_gate_enabled_ &&
      contract->kind == OvertakeOverrideContractKind::
                            SPATIAL_LATERAL_AND_SPEED_V4) {
    pending_state_lattice_v4_contract_ = PendingStateLatticeV4Contract{
        contract.value(), std::move(source_payload), std::move(source_wire),
        now_sec};
    if (state_lattice_v2_control_trajectory_cache_.has_value() &&
        state_lattice_v2_control_trajectory_cache_->trajectory != nullptr &&
        state_lattice_v2_control_trajectory_cache_->identity.plan_generation ==
            contract->generation) {
      (void)activatePendingStateLatticeV4Contract(
          state_lattice_v2_control_trajectory_cache_->identity);
    }
    return;
  }
  overtake_source_payload_ = std::move(source_payload);
  aw2_overtake_source_wire_ = std::move(source_wire);
  overtake_source_generation_ = contract->generation;
  overtake_speed_only_latch_.observeValid(contract.value());
  applyReceivedOvertakeOverride(contract.value());
  last_overtake_override_sec_ = now_sec;
  last_valid_override_contract_sec_ = now_sec;
  last_valid_override_contract_generation_ = contract->generation;
  last_valid_override_contract_inactive_ = false;
  last_valid_override_contract_received_ = true;
}

bool SimplePurePursuit::activatePendingStateLatticeV4Contract(
    const multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity &identity) {
  if (!pending_state_lattice_v4_contract_.has_value() ||
      pending_state_lattice_v4_contract_->contract.generation !=
          identity.plan_generation ||
      pending_state_lattice_v4_contract_->contract.kind !=
          OvertakeOverrideContractKind::SPATIAL_LATERAL_AND_SPEED_V4) {
    return false;
  }
  auto pending = std::move(pending_state_lattice_v4_contract_.value());
  pending_state_lattice_v4_contract_.reset();
  overtake_source_payload_ = std::move(pending.source_payload);
  aw2_overtake_source_wire_ = std::move(pending.source_wire);
  overtake_source_generation_ = pending.contract.generation;
  overtake_speed_only_latch_.observeValid(pending.contract);
  applyReceivedOvertakeOverride(pending.contract);
  last_overtake_override_sec_ = pending.receive_steady_sec;
  last_valid_override_contract_sec_ = pending.receive_steady_sec;
  last_valid_override_contract_generation_ = pending.contract.generation;
  last_valid_override_contract_inactive_ = false;
  last_valid_override_contract_received_ = true;
  return true;
}

bool SimplePurePursuit::stateLatticeV4RendezvousPending() const {
  if (!state_lattice_v4_poc_command_activation_enabled_ ||
      !state_lattice_v4_poc_identity_gate_enabled_) {
    return false;
  }
  const double now_sec = steadyNowSec();
  const auto fresh = [this, now_sec](double receive_sec) {
    const double age_sec = inputAgeSec(receive_sec, now_sec);
    return ageFresh(age_sec, overtake_override_timeout_sec_) &&
           ageFresh(age_sec, max_override_age_sec_);
  };
  const auto newer_generation = [](std::uint32_t current,
                                   std::uint32_t candidate) {
    if (current == 0U || candidate == 0U || current == candidate) {
      return false;
    }
    constexpr std::uint32_t kMaxGeneration = 16777215U;
    constexpr std::uint32_t kHalfRange = kMaxGeneration / 2U;
    const std::uint32_t distance = candidate > current
                                       ? candidate - current
                                       : (kMaxGeneration - current) + candidate;
    return distance > 0U && distance <= kHalfRange;
  };
  const bool v2_arrived_first =
      state_lattice_v2_control_trajectory_cache_.has_value() &&
      fresh(state_lattice_v2_control_trajectory_cache_->receive_steady_sec) &&
      newer_generation(overtake_override_generation_,
                       state_lattice_v2_control_trajectory_cache_
                           ->identity.plan_generation);
  const bool v4_arrived_first =
      pending_state_lattice_v4_contract_.has_value() &&
      fresh(pending_state_lattice_v4_contract_->receive_steady_sec) &&
      newer_generation(
          overtake_override_generation_,
          pending_state_lattice_v4_contract_->contract.generation);
  return v2_arrived_first || v4_arrived_first;
}

void SimplePurePursuit::applyReceivedOvertakeOverride(
    const OvertakeOverrideContract &contract) {
  overtake_lateral_offsets_ = contract.lateral_offsets;
  overtake_speed_caps_ = contract.speed_caps;
  overtake_longitudinal_offsets_m_ = contract.longitudinal_offsets_m;
  overtake_lateral_override_active_ =
      contract.kind == OvertakeOverrideContractKind::LATERAL_AND_SPEED_V1 ||
      contract.kind == OvertakeOverrideContractKind::LATERAL_AND_SPEED_V3 ||
      contract.kind ==
          OvertakeOverrideContractKind::SPATIAL_LATERAL_AND_SPEED_V4;
  overtake_speed_only_active_ =
      contract.kind == OvertakeOverrideContractKind::SPEED_ONLY_V2;
  overtake_solver_horizon_authorized_ = contract.solver_horizon_authorized;
  overtake_mandatory_lateral_avoidance_ = contract.mandatory_lateral_avoidance;
  overtake_override_generation_ = contract.generation;
  overtake_override_contract_kind_ = contract.kind;
  overtake_mode_id_ = contract.mode_id;
  overtake_override_active_ = true;
}

void SimplePurePursuit::onMpcPredictedHorizonContract(
    const String::SharedPtr msg) {
  const auto version = jsonIntegerField(msg->data, "contract_version");
  const auto stamp_sec = jsonIntegerField(msg->data, "horizon_stamp_sec");
  const auto stamp_nanosec =
      jsonIntegerField(msg->data, "horizon_stamp_nanosec");
  const auto source = jsonStringField(msg->data, "source");
  const auto mode_id = jsonIntegerField(msg->data, "mode_id");
  const auto generation = jsonIntegerField(msg->data, "override_generation");
  const auto solver_horizon_authorized =
      jsonBooleanField(msg->data, "solver_horizon_authorized");
  const auto mandatory_lateral_avoidance =
      jsonBooleanField(msg->data, "mandatory_lateral_avoidance");
  if (!version.has_value() || version.value() != 2 || !stamp_sec.has_value() ||
      !stamp_nanosec.has_value() || !source.has_value() ||
      !mode_id.has_value() || !generation.has_value() ||
      !solver_horizon_authorized.has_value() ||
      !mandatory_lateral_avoidance.has_value() ||
      stamp_sec.value() < std::numeric_limits<std::int32_t>::min() ||
      stamp_sec.value() > std::numeric_limits<std::int32_t>::max() ||
      stamp_nanosec.value() < 0 || stamp_nanosec.value() >= 1000000000LL ||
      mode_id.value() < 0 || mode_id.value() > 255 || generation.value() < 0 ||
      generation.value() > kMaxOvertakeOverrideGeneration) {
    mpc_horizon_contract_received_ = false;
    return;
  }
  mpc_horizon_contract_stamp_sec_ =
      static_cast<std::int32_t>(stamp_sec.value());
  mpc_horizon_contract_stamp_nanosec_ =
      static_cast<std::uint32_t>(stamp_nanosec.value());
  mpc_horizon_contract_source_ = *source;
  mpc_horizon_contract_mode_id_ = static_cast<int>(mode_id.value());
  mpc_horizon_contract_generation_ =
      static_cast<std::uint32_t>(generation.value());
  mpc_horizon_contract_solver_horizon_authorized_ =
      solver_horizon_authorized.value();
  mpc_horizon_contract_mandatory_lateral_avoidance_ =
      mandatory_lateral_avoidance.value();
  mpc_horizon_contract_received_ = true;
  last_mpc_predicted_horizon_contract_receive_sec_ = steadyNowSec();
}

void SimplePurePursuit::onMpcHealth(const String::SharedPtr msg) {
  last_mpc_health_receive_sec_ = steadyNowSec();

  if (const auto status = jsonStringField(msg->data, "mpc_status")) {
    mpc_health_status_ = *status;
  } else {
    mpc_health_status_ = "parse_error";
  }

  if (const auto infeasible_count =
          jsonNumberField(msg->data, "mpc_infeasible_count")) {
    mpc_health_infeasible_count_ =
        std::max(0, static_cast<int>(std::llround(*infeasible_count)));
  } else {
    mpc_health_infeasible_count_ = 0;
  }

  if (const auto horizon_source =
          jsonStringField(msg->data, "mpc_predicted_horizon_source")) {
    mpc_predicted_horizon_source_ = *horizon_source;
  } else {
    mpc_predicted_horizon_source_ = "unknown";
  }
}

void SimplePurePursuit::onRecoveryStatus(const RecoveryStatus::SharedPtr msg) {
  recovery_status_ = msg;
  last_recovery_status_receive_sec_ = steadyNowSec();
}

bool SimplePurePursuit::recoveryStatusAllowsControl(
    double now_sec, const Trajectory &control_trajectory,
    std::string *reason) const {
  if (!recovery_mode_) {
    if (reason != nullptr) {
      *reason = "not_recovery_mode";
    }
    return false;
  }
  if (!recovery_status_ || !last_recovery_status_receive_sec_.has_value()) {
    if (reason != nullptr) {
      *reason = "missing_recovery_status";
    }
    return false;
  }
  const double status_age_sec =
      inputAgeSec(last_recovery_status_receive_sec_, now_sec);
  if (!ageFresh(status_age_sec, recovery_status_timeout_sec_)) {
    if (reason != nullptr) {
      *reason = "stale_recovery_status";
    }
    return false;
  }
  const bool active_state =
      recovery_status_->state == RecoveryStatus::ACTIVE ||
      recovery_status_->state == RecoveryStatus::HANDOFF_VERIFY;
  if (!active_state) {
    if (reason != nullptr) {
      *reason = "recovery_state_not_active";
    }
    return false;
  }
  if (!recovery_status_->input_complete ||
      !recovery_status_->trajectory_valid ||
      !recovery_status_->trajectory_safe || recovery_status_->stop_required) {
    if (reason != nullptr) {
      *reason = "recovery_status_not_safe";
    }
    return false;
  }
  const auto &expected = recovery_status_->trajectory_header;
  const auto &actual = control_trajectory.header;
  if (expected.frame_id != actual.frame_id ||
      expected.stamp.sec != actual.stamp.sec ||
      expected.stamp.nanosec != actual.stamp.nanosec) {
    if (reason != nullptr) {
      *reason = "recovery_trajectory_header_mismatch";
    }
    return false;
  }
  if (reason != nullptr) {
    *reason = "ok";
  }
  return true;
}

void SimplePurePursuit::publishRecoveryControlCommand(
    const AckermannControlCommand &cmd, const Trajectory &control_trajectory) {
  if (!recovery_status_) {
    return;
  }
  RecoveryControlCommand msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = control_trajectory.header.frame_id;
  msg.attempt_id = recovery_status_->attempt_id;
  msg.trajectory_generation = recovery_status_->trajectory_generation;
  msg.trajectory_header = control_trajectory.header;
  msg.command = cmd;
  pub_recovery_cmd_->publish(msg);
}

void SimplePurePursuit::clearOvertakeOverride() {
  overtake_speed_only_latch_.clear();
  overtake_override_active_ = false;
  overtake_lateral_override_active_ = false;
  overtake_speed_only_active_ = false;
  overtake_solver_horizon_authorized_ = false;
  overtake_mandatory_lateral_avoidance_ = false;
  overtake_mode_id_ = 0;
  overtake_override_generation_ = 0;
  overtake_override_contract_kind_ = OvertakeOverrideContractKind::INACTIVE;
  overtake_lateral_offsets_.clear();
  overtake_speed_caps_.clear();
  overtake_longitudinal_offsets_m_.clear();
  last_overtake_override_sec_ = -1.0e9;
}

bool SimplePurePursuit::hasLatchedSpeedOnlyCap() const {
  return overtake_speed_only_active_ &&
         overtake_speed_only_latch_.retained().has_value();
}

bool SimplePurePursuit::overtakeOverrideFresh(double now_sec) const {
  if (!use_overtake_reference_override_ || !overtake_override_active_ ||
      overtake_speed_caps_.empty() ||
      (overtake_lateral_override_active_ &&
       overtake_lateral_offsets_.empty()) ||
      (!overtake_lateral_override_active_ && !overtake_speed_only_active_)) {
    return false;
  }
  const double age_sec = inputAgeSec(last_overtake_override_sec_, now_sec);
  return ageFresh(age_sec, overtake_override_timeout_sec_) &&
         ageFresh(age_sec, max_override_age_sec_);
}

bool SimplePurePursuit::applyOvertakeOverride(
    Trajectory &trajectory, std::size_t nearest_traj_point_idx, double now_sec,
    double *applied_spatial_arc_m, double *required_spatial_arc_m,
    std::string *apply_reason) {
  if (applied_spatial_arc_m != nullptr) {
    *applied_spatial_arc_m = std::numeric_limits<double>::quiet_NaN();
  }
  if (required_spatial_arc_m != nullptr) {
    *required_spatial_arc_m = std::numeric_limits<double>::quiet_NaN();
  }
  if (apply_reason != nullptr) {
    *apply_reason = "override_not_fresh_or_lateral_inactive";
  }
  if (!overtakeOverrideFresh(now_sec) || !overtake_lateral_override_active_ ||
      nearest_traj_point_idx >= trajectory.points.size()) {
    return false;
  }

  if (state_lattice_v4_poc_identity_gate_enabled_ &&
      (!state_lattice_v4_poc_identity_.has_value() ||
       !v4PocIdentityMatches(
           state_lattice_v4_poc_identity_.value(),
           overtake_override_generation_, trajectory.header.frame_id,
           trajectory.header.stamp, reference_source_generation_))) {
    if (apply_reason != nullptr) {
      *apply_reason = "v4_poc_identity_mismatch";
    }
    return false;
  }

  const bool spatial_profile = !overtake_longitudinal_offsets_m_.empty();
  std::optional<std::size_t> spatial_endpoint_index;
  if (spatial_profile) {
    const double spatial_endpoint_arc_m =
        overtake_longitudinal_offsets_m_.back();
    spatial_endpoint_index = insertSpatialHorizonEndpoint(
        trajectory, nearest_traj_point_idx, spatial_endpoint_arc_m);
    if (!spatial_endpoint_index.has_value()) {
      if (apply_reason != nullptr) {
        *apply_reason = "spatial_endpoint_unavailable";
      }
      return false;
    }
    double covered_arc_m = 0.0;
    std::size_t covered_point_count = 1U;
    double previous_x =
        trajectory.points[nearest_traj_point_idx].pose.position.x;
    double previous_y =
        trajectory.points[nearest_traj_point_idx].pose.position.y;
    for (std::size_t i = 1U;
         nearest_traj_point_idx + i < trajectory.points.size(); ++i) {
      const auto &point = trajectory.points[nearest_traj_point_idx + i];
      const double segment_m = std::hypot(point.pose.position.x - previous_x,
                                          point.pose.position.y - previous_y);
      previous_x = point.pose.position.x;
      previous_y = point.pose.position.y;
      if (covered_arc_m + segment_m >
          overtake_longitudinal_offsets_m_.back() + 1.0e-9) {
        break;
      }
      covered_arc_m += segment_m;
      covered_point_count = i + 1U;
      if (nearest_traj_point_idx + i == spatial_endpoint_index.value()) {
        break;
      }
    }
    // Reaccumulation is performed before any lateral or speed mutation. This
    // proves that the exact existing/inserted point still represents the
    // SafetyEvaluator-authorized profile endpoint within a fixed absolute
    // tolerance; an endpoint mismatch fails closed without a partly shifted
    // trajectory.
    const auto endpoint_lateral = sampleOvertakeProfileAtProvenEndpoint(
        overtake_longitudinal_offsets_m_, overtake_lateral_offsets_,
        covered_arc_m, spatial_endpoint_index.value(),
        spatial_endpoint_index.value());
    const auto endpoint_speed = sampleOvertakeProfileAtProvenEndpoint(
        overtake_longitudinal_offsets_m_, overtake_speed_caps_, covered_arc_m,
        spatial_endpoint_index.value(), spatial_endpoint_index.value());
    if (covered_point_count !=
            spatial_endpoint_index.value() - nearest_traj_point_idx + 1U ||
        !endpoint_lateral.has_value() || !endpoint_speed.has_value()) {
      if (apply_reason != nullptr) {
        *apply_reason = "spatial_endpoint_reaccumulation_mismatch";
      }
      return false;
    }
    // The exact trajectory point provenance and reaccumulated distance were
    // proven above. Use the SafetyEvaluator-authorized nominal profile endpoint
    // for all subsequent horizon bookkeeping instead of propagating floating
    // point accumulation noise.
    covered_arc_m = spatial_endpoint_arc_m;
    const double current_speed_mps =
        odometry_ != nullptr ? std::abs(odometry_->twist.twist.linear.x)
                             : std::numeric_limits<double>::quiet_NaN();
    const double target_speed_mps =
        !overtake_speed_caps_.empty() &&
                std::isfinite(overtake_speed_caps_.front())
            ? std::max(0.0, overtake_speed_caps_.front())
            : 0.0;
    const LookaheadParams lookahead_params{
        lookahead_gain_, lookahead_min_distance_,
        curvature_adaptive_lookahead_enabled_,
        curvature_lookahead_min_distance_, curvature_lookahead_sensitivity_};
    double required_lookahead_m = speedBasedLookaheadDistance(
        target_speed_mps, current_speed_mps, lookahead_params);
    if (has_smoothed_lookahead_distance_ &&
        std::isfinite(smoothed_lookahead_distance_)) {
      required_lookahead_m =
          std::max(required_lookahead_m, smoothed_lookahead_distance_);
    }
    const double minimum_required_arc_m = minimumExecutableSpatialHorizonArc(
        required_lookahead_m, current_speed_mps,
        overtake_spatial_horizon_min_arc_m_,
        overtake_spatial_horizon_min_time_sec_,
        overtake_spatial_horizon_response_delay_sec_,
        overtake_spatial_horizon_brake_decel_mps2_,
        overtake_short_spatial_horizon_v_max_mps_);
    if (applied_spatial_arc_m != nullptr) {
      *applied_spatial_arc_m = covered_arc_m;
    }
    if (required_spatial_arc_m != nullptr) {
      *required_spatial_arc_m = minimum_required_arc_m;
    }
    if (!spatialOverrideHorizonSufficient(covered_point_count, covered_arc_m,
                                          minimum_required_arc_m)) {
      const double safe_cap_mps =
          std::isfinite(overtake_short_spatial_horizon_v_max_mps_)
              ? std::clamp(overtake_short_spatial_horizon_v_max_mps_, 1.0e-3,
                           0.20)
              : 0.20;
      // これは受信したtransport v2ではなく、この周期だけのローカルfail-safe。
      // latchへ保存すると、後続のfresh v4や一時的な不正payload後も0.2 m/sが
      // 残り続けるため、真正なplanner発行v2だけを保持対象にする。
      overtake_lateral_override_active_ = false;
      overtake_speed_only_active_ = true;
      overtake_solver_horizon_authorized_ = false;
      overtake_mandatory_lateral_avoidance_ = false;
      overtake_lateral_offsets_.clear();
      overtake_longitudinal_offsets_m_.clear();
      overtake_speed_caps_ = {safe_cap_mps};
      if (apply_reason != nullptr) {
        *apply_reason = "insufficient_verified_spatial_horizon";
      }
      return false;
    }
  }
  const std::size_t count =
      spatial_profile
          ? spatial_endpoint_index.value() - nearest_traj_point_idx + 1U
          : std::min(overtake_lateral_offsets_.size(),
                     trajectory.points.size() - nearest_traj_point_idx);
  double path_distance_m = 0.0;
  double previous_x = trajectory.points[nearest_traj_point_idx].pose.position.x;
  double previous_y = trajectory.points[nearest_traj_point_idx].pose.position.y;
  std::size_t applied_count = 0U;
  bool spatial_endpoint_applied = false;
  for (std::size_t i = 0; i < count; ++i) {
    auto &point = trajectory.points[nearest_traj_point_idx + i];
    const double original_x = point.pose.position.x;
    const double original_y = point.pose.position.y;
    if (i > 0U) {
      path_distance_m +=
          std::hypot(original_x - previous_x, original_y - previous_y);
    }
    previous_x = original_x;
    previous_y = original_y;

    if (spatial_profile &&
        path_distance_m > overtake_longitudinal_offsets_m_.back() +
                              kSpatialProfileEndpointToleranceM) {
      // SafetyEvaluatorが検証した空間horizonより先の基準線を残すと、
      // lookaheadが未評価区間を参照する。最初の範囲外点から先を切り、
      // 制御対象を検証済み区間内へ限定する。
      trajectory.points.resize(nearest_traj_point_idx + i);
      break;
    }

    const std::size_t trajectory_index = nearest_traj_point_idx + i;
    const bool proven_spatial_endpoint =
        spatial_profile && trajectory_index == spatial_endpoint_index.value();
    const auto sampled_offset =
        spatial_profile
            ? (proven_spatial_endpoint
                   ? sampleOvertakeProfileAtProvenEndpoint(
                         overtake_longitudinal_offsets_m_,
                         overtake_lateral_offsets_, path_distance_m,
                         trajectory_index, spatial_endpoint_index.value())
                   : sampleOvertakeProfileByDistance(
                         overtake_longitudinal_offsets_m_,
                         overtake_lateral_offsets_, path_distance_m))
            : std::optional<double>{overtake_lateral_offsets_[i]};
    const double offset_m =
        sampled_offset.value_or(std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(offset_m)) {
      continue;
    }

    const double yaw = tf2::getYaw(point.pose.orientation);
    point.pose.position.x = point.pose.position.x - offset_m * std::sin(yaw);
    point.pose.position.y = point.pose.position.y + offset_m * std::cos(yaw);

    const auto speed_cap =
        spatial_profile
            ? (proven_spatial_endpoint
                   ? sampleOvertakeProfileAtProvenEndpoint(
                         overtake_longitudinal_offsets_m_, overtake_speed_caps_,
                         path_distance_m, trajectory_index,
                         spatial_endpoint_index.value())
                   : sampleOvertakeProfileByDistance(
                         overtake_longitudinal_offsets_m_, overtake_speed_caps_,
                         path_distance_m))
            : overtakeSpeedCap(i, now_sec);
    if (speed_cap.has_value()) {
      point.longitudinal_velocity_mps =
          std::min(point.longitudinal_velocity_mps,
                   static_cast<float>(speed_cap.value()));
    }
    applied_count = i + 1U;
    spatial_endpoint_applied =
        spatial_endpoint_applied || proven_spatial_endpoint;
  }
  if (spatial_profile && spatial_endpoint_applied) {
    trajectory.points.resize(spatial_endpoint_index.value() + 1U);
  }
  const bool applied =
      applied_count > 0U && (!spatial_profile || spatial_endpoint_applied);
  if (apply_reason != nullptr) {
    *apply_reason = applied ? "applied" : "no_finite_lateral_samples";
  }
  return applied;
}

std::optional<double>
SimplePurePursuit::overtakeSpeedCap(std::size_t horizon_index,
                                    double now_sec) const {
  if (!overtakeOverrideFresh(now_sec) && !hasLatchedSpeedOnlyCap()) {
    return std::nullopt;
  }
  const std::size_t speed_cap_index =
      overtake_speed_only_active_ ? 0 : horizon_index;
  if (speed_cap_index >= overtake_speed_caps_.size()) {
    return std::nullopt;
  }
  const double speed_cap_mps = overtake_speed_caps_[speed_cap_index];
  if (!std::isfinite(speed_cap_mps) || speed_cap_mps <= 0.0) {
    return std::nullopt;
  }
  return speed_cap_mps;
}

double
SimplePurePursuit::overtakeLateralOffset(std::size_t horizon_index) const {
  if (!overtake_lateral_override_active_ ||
      horizon_index >= overtake_lateral_offsets_.size()) {
    return 0.0;
  }
  const double offset_m = overtake_lateral_offsets_[horizon_index];
  return std::isfinite(offset_m) ? offset_m : 0.0;
}

ControllerTrackingStatus SimplePurePursuit::publishControllerTrackingStatus(
    const rclcpp::Time &stamp, const ControlTrajectoryContext *context,
    const AckermannControlCommand *command,
    const LongitudinalCommand *longitudinal, const LateralCommand *lateral,
    double now_sec, const ControlCyclePlanSnapshot *plan_snapshot,
    bool state_lattice_delivery_gap) {
  const bool command_finite =
      command != nullptr && std::isfinite(command->longitudinal.speed) &&
      std::isfinite(command->longitudinal.acceleration) &&
      std::isfinite(command->lateral.steering_tire_angle);
  const double contract_age_sec = now_sec - last_valid_override_contract_sec_;
  const bool v4_cartesian_contract_fresh =
      use_overtake_reference_override_ &&
      last_valid_override_contract_received_ &&
      !last_valid_override_contract_inactive_ && context != nullptr &&
      context->source == "state_lattice_v4_cartesian" &&
      context->v4_poc_geometry_applied &&
      context->selected_cartesian_proposal != nullptr &&
      last_valid_override_contract_generation_ ==
          context->selected_cartesian_proposal->identity.plan_generation &&
      overtake_override_generation_ ==
          context->selected_cartesian_proposal->identity.plan_generation &&
      ageFresh(contract_age_sec, overtake_override_timeout_sec_) &&
      ageFresh(contract_age_sec, max_override_age_sec_);
  const bool contract_fresh =
      use_overtake_reference_override_ &&
      last_valid_override_contract_received_ &&
      last_valid_override_contract_generation_ > 0U &&
      plan_snapshot != nullptr && plan_snapshot->plan != nullptr &&
      last_valid_override_contract_generation_ ==
          plan_snapshot->plan->plan_generation &&
      ageFresh(contract_age_sec, overtake_override_timeout_sec_) &&
      ageFresh(contract_age_sec, max_override_age_sec_);
  const bool inactive_contract_applied =
      contract_fresh && last_valid_override_contract_inactive_ &&
      context != nullptr && !overtake_override_active_ &&
      !context->overtake_override_applied;
  const bool lateral_override_applied =
      v4_cartesian_contract_fresh ||
      (contract_fresh && !last_valid_override_contract_inactive_ &&
       context != nullptr && !overtake_speed_only_active_ &&
       ((context->overtake_override_applied && overtake_override_active_ &&
         overtake_override_generation_ ==
             last_valid_override_contract_generation_) ||
        (context->mpc_horizon_applied &&
         context->applied_horizon_generation ==
             last_valid_override_contract_generation_ &&
         context->applied_horizon_source == "solver_prediction")));
  const bool speed_only_contract_applied = speedOnlyTrackingContractApplied(
      contract_fresh, last_valid_override_contract_inactive_,
      overtake_speed_only_active_, overtake_lateral_override_active_,
      overtake_override_generation_, last_valid_override_contract_generation_,
      longitudinal != nullptr ? longitudinal->overtake_speed_cap_mps : 0.0,
      command != nullptr ? command->longitudinal.speed
                         : std::numeric_limits<double>::quiet_NaN());
  // Angle/rate limiting is the normal actuator-safe PP output path.  A finite
  // command bounded by valid hard limits remains executable; convergence of
  // measured steering is diagnostic evidence, not a prerequisite for starting
  // longitudinal motion on the candidate's already-limited entry speed.
  const bool steering_execution_usable =
      lateral != nullptr && lateral->steering_limits_valid;
  const bool v2_control_applied =
      state_lattice_v2_command_activation_enabled_ && context != nullptr &&
      context->source == "state_lattice_v2" &&
      state_lattice_v2_control_trajectory_cache_.has_value();
  const bool v4_cartesian_control_applied =
      context != nullptr && context->source == "state_lattice_v4_cartesian" &&
      context->v4_poc_geometry_applied &&
      context->selected_cartesian_proposal != nullptr;
  const bool tracking_usable =
      !recovery_mode_ && command_finite && context != nullptr &&
      context->valid && steering_execution_usable &&
      (v2_control_applied || inactive_contract_applied ||
       speed_only_contract_applied ||
       lateral_override_applied);

  const bool pass_warmup_plan_matches =
      command_finite && context != nullptr && lateral != nullptr &&
      lateral_override_applied && plan_snapshot != nullptr &&
      plan_snapshot->plan != nullptr &&
      context->selected_cartesian_proposal != nullptr &&
      lateral->measured_steering_fresh &&
      plan_snapshot->plan->plan_generation ==
          context->selected_cartesian_proposal->identity.plan_generation &&
      plan_snapshot->plan->trajectory_authorized &&
      plan_snapshot->plan->lateral_maneuver_required &&
      plan_snapshot->plan->lateral_stop_authority_kind ==
          OvertakePlan::LATERAL_STOP_PASS_WARMUP &&
      plan_snapshot->plan->lateral_stop_transaction_pass_direction != 0 &&
      plan_snapshot->plan->lateral_stop_authority_token != 0U;
  const double measured_tolerance_rad =
      std::max(1.0e-6, free_run_live_exact_hard_steering_rate_limit_radps_ *
                           steering_command_nominal_dt_sec_);
  const auto pass_warmup_acquisition =
      pass_warmup_plan_matches
          ? evaluatePassWarmupSteeringAcquisition(
                lateral->requested_output_steering_tire_angle_rad,
                lateral->steering_tire_angle_rad,
                lateral->raw_steering_tire_angle_rad,
                lateral->measured_steering_rad,
                lateral->measured_steering_age_sec,
                steering_status_timeout_sec_, 1.0e-9,
                measured_tolerance_rad)
          : PassWarmupSteeringAcquisition{};
  ControllerTrackingStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = "base_link";
  status.plan_generation =
      (v2_control_applied || v4_cartesian_control_applied)
          ? state_lattice_v2_control_trajectory_cache_->identity.plan_generation
          : (plan_snapshot != nullptr && plan_snapshot->plan != nullptr
                 ? plan_snapshot->plan->plan_generation
                 : 0U);
  status.mpc_horizon_usable =
      context != nullptr && context->mpc_horizon_applied;
  status.pp_command_fresh = command_finite;
  status.trajectory_tracking_usable = tracking_usable;
  status.pass_warmup_steering_acquisition_active =
      pass_warmup_plan_matches && pass_warmup_acquisition.evidence_valid &&
      !pass_warmup_acquisition.motion_ready;
  status.pass_warmup_requested_steering_limited =
      pass_warmup_plan_matches &&
      pass_warmup_acquisition.acquisition_required;
  status.pass_warmup_measured_steering_converged =
      pass_warmup_plan_matches &&
      pass_warmup_acquisition.measured_steering_converged;
  status.pass_warmup_motion_ready =
      pass_warmup_plan_matches && pass_warmup_acquisition.motion_ready;
  // STOP transportのN-1継続性はplan/constraint履歴を所有するMuxだけが判定する。
  status.safety_constraint_release_ready = false;
  status.attack_follow_stop_transport_release_ready = false;
  status.pp_command_binding_valid = command_finite;
  status.lateral_stop_authority_kind = OvertakePlan::LATERAL_STOP_NONE;
  status.lateral_stop_transaction_pass_direction = 0;
  status.lateral_stop_authority_token = 0U;
  const bool typed_lateral_stop_plan_applied =
      (tracking_usable || status.pass_warmup_steering_acquisition_active) &&
      lateral_override_applied && plan_snapshot != nullptr &&
      plan_snapshot->plan != nullptr &&
      plan_snapshot->plan->plan_generation == status.plan_generation &&
      plan_snapshot->plan->trajectory_authorized &&
      plan_snapshot->plan->lateral_maneuver_required &&
      (plan_snapshot->plan->lateral_stop_authority_kind ==
           OvertakePlan::LATERAL_STOP_CURRENT_D_HOLD ||
       plan_snapshot->plan->lateral_stop_authority_kind ==
           OvertakePlan::LATERAL_STOP_PASS_WARMUP) &&
      plan_snapshot->plan->lateral_stop_transaction_pass_direction != 0 &&
      plan_snapshot->plan->lateral_stop_authority_token != 0U;
  if (typed_lateral_stop_plan_applied) {
    status.lateral_stop_authority_kind =
        plan_snapshot->plan->lateral_stop_authority_kind;
    status.lateral_stop_transaction_pass_direction =
        plan_snapshot->plan->lateral_stop_transaction_pass_direction;
    status.lateral_stop_authority_token =
        plan_snapshot->plan->lateral_stop_authority_token;
  }
  status.pp_command_speed_mps = command_finite
                                    ? command->longitudinal.speed
                                    : std::numeric_limits<float>::quiet_NaN();
  status.pp_command_acceleration_mps2 =
      command_finite ? command->longitudinal.acceleration
                     : std::numeric_limits<float>::quiet_NaN();
  status.pp_command_steering_tire_angle_rad =
      command_finite ? command->lateral.steering_tire_angle
                     : std::numeric_limits<float>::quiet_NaN();
  status.command_age_sec =
      command_finite ? 0.0F : std::numeric_limits<float>::infinity();
  if (tracking_usable) {
    status.reason = "ready";
  } else if (recovery_mode_) {
    status.reason = "recovery_mode";
  } else if (state_lattice_delivery_gap && !command_finite) {
    status.reason = "override_contract_missing_or_stale";
  } else if (!command_finite) {
    status.reason = "command_missing_or_nonfinite";
  } else if (context == nullptr || !context->valid) {
    status.reason = "trajectory_context_invalid";
  } else if (!steering_execution_usable) {
    status.reason = lateral != nullptr && lateral->steering_limits_valid
                        ? "authorized_trajectory_requires_steering_saturation"
                        : "steering_command_limit_invalid";
  } else if (!v2_control_applied && !contract_fresh &&
             !v4_cartesian_contract_fresh) {
    status.reason = "override_contract_missing_or_stale";
  } else if (!inactive_contract_applied && !speed_only_contract_applied &&
             !lateral_override_applied) {
    status.reason = "override_contract_not_applied";
  } else {
    status.reason = "tracking_unusable";
  }
  if (pub_tracking_status_) {
    pub_tracking_status_->publish(status);
  }
  return status;
}

std::optional<ControllerCommandEnvelope>
SimplePurePursuit::publishControllerCommandEnvelope(
    const AckermannControlCommand &command,
    const ControllerTrackingStatus &tracking_status,
    const ControlCyclePlanSnapshot *plan_snapshot,
    const ControlTrajectoryContext *context) {
  if (!pub_command_envelope_) {
    return std::nullopt;
  }
  if (command_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    RCLCPP_ERROR(get_logger(),
                 "PurePursuit command envelope sequence exhausted");
    return std::nullopt;
  }
  ++command_sequence_;
  const auto envelope = makeControllerCommandEnvelopeV1(
      command, tracking_status, producer_instance_id_, command_sequence_,
      plan_snapshot != nullptr ? plan_snapshot->plan.get() : nullptr,
      context != nullptr ? context->selected_cartesian_proposal.get()
                         : nullptr);
  pub_command_envelope_->publish(envelope);
  return envelope;
}

void SimplePurePursuit::publishControllerExecutionEnvelope(
    const ControllerCommandEnvelope &command_envelope,
    const ControlTrajectoryContext *context,
    const ControlPosePrediction *control_pose, const LateralCommand *lateral,
    double now_sec, const ControlCyclePlanSnapshot *plan_snapshot) {
  if (!pub_execution_envelope_) {
    return;
  }

  ControllerExecutionWitnessInput input;
  input.header = command_envelope.header;
  input.controller_role = recovery_mode_
                              ? ControllerExecutionWitness::ROLE_RECOVERY
                              : ControllerExecutionWitness::ROLE_PRIMARY;
  if (context != nullptr && context->trajectory != nullptr &&
      context->base_trajectory != nullptr && control_pose != nullptr) {
    input.base_trajectory = *context->base_trajectory;
    input.applied_trajectory = *context->trajectory;
    input.reference_stamp = input.base_trajectory.header.stamp;
    input.source_stamp = input.applied_trajectory.header.stamp;
    input.nearest_trajectory_index = context->nearest_index;
    input.control_pose.position = control_pose->position;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, control_pose->yaw);
    input.control_pose.orientation.x = orientation.x();
    input.control_pose.orientation.y = orientation.y();
    input.control_pose.orientation.z = orientation.z();
    input.control_pose.orientation.w = orientation.w();
    input.raw_steering_tire_angle_rad =
        lateral != nullptr
            ? lateral->requested_output_steering_tire_angle_rad
            : command_envelope.command.lateral.steering_tire_angle;
    input.bounded_steering_tire_angle_rad =
        command_envelope.command.lateral.steering_tire_angle;
    if (context->mpc_horizon_applied) {
      input.trajectory_source =
          ControllerExecutionWitness::SOURCE_PREDICTED_HORIZON;
      input.source_generation = context->applied_horizon_generation;
    } else if (context->overtake_override_applied) {
      input.trajectory_source =
          ControllerExecutionWitness::SOURCE_REFERENCE_OVERRIDE;
      input.source_generation = overtake_source_generation_;
      input.source_payload = overtake_source_payload_;
    } else {
      input.trajectory_source =
          ControllerExecutionWitness::SOURCE_REFERENCE_TRAJECTORY;
    }
    input.available_spatial_horizon_m = 0.0;
    for (std::size_t i = context->nearest_index + 1U;
         i < input.applied_trajectory.points.size(); ++i) {
      const auto &previous =
          input.applied_trajectory.points[i - 1U].pose.position;
      const auto &current = input.applied_trajectory.points[i].pose.position;
      input.available_spatial_horizon_m +=
          std::hypot(current.x - previous.x, current.y - previous.y);
    }
    input.required_spatial_horizon_m =
        context->overtake_spatial_horizon_required_arc_m;
    if (!std::isfinite(input.required_spatial_horizon_m) ||
        input.required_spatial_horizon_m <= 0.0) {
      input.required_spatial_horizon_m = lookahead_min_distance_;
    }
    // PP does not own the actuator hard angle/rate limits or a conservative
    // closed-loop swept rollout. Leave both unavailable, so AW1 publishes the
    // atomic geometry witness but never claims it is release-usable.
  }

  const bool typed_plan_fresh =
      plan_snapshot != nullptr && plan_snapshot->plan != nullptr &&
      ageFresh(now_sec - plan_snapshot->receive_steady_sec,
               max_override_age_sec_);
  PurePursuitExactSnapshotInput exact;
  exact.enabled = pp_core_exact_snapshot_enabled_ && context != nullptr &&
                  context->overtake_override_applied &&
                  control_pose != nullptr && lateral != nullptr &&
                  free_run_source_key_valid_;
  if (exact.enabled) {
    exact.base_source_binding_valid = free_run_source_key_valid_;
    exact.base_source_key = free_run_source_key_;
    exact.control_pose_stamp = command_envelope.header.stamp;
    exact.diagnostic_lease_duration_sec = 0.05;
    exact.selected_lookahead_trajectory_index =
        lateral->lookahead_selected_trajectory_index;
    exact.lookahead_endpoint_fallback = lateral->lookahead_endpoint_fallback;
    exact.control_speed_mps = std::max(0.0, control_pose->velocity_mps);
    exact.resolved_lookahead_distance_m = lateral->lookahead_distance_m;
    exact.geometric_steering_tire_angle_rad =
        lateral->pure_pursuit_steering_tire_angle_rad;
    exact.curvature_feedforward_steering_rad =
        lateral->curvature_feedforward_steering_rad;
    exact.raw_steering_tire_angle_rad = lateral->raw_steering_tire_angle_rad;
    exact.requested_output_steering_tire_angle_rad =
        lateral->requested_output_steering_tire_angle_rad;
    exact.bounded_steering_tire_angle_rad = lateral->steering_tire_angle_rad;
    exact.requested_steering_tire_rotation_rate_radps =
        lateral->requested_steering_tire_rotation_rate_radps;
    exact.bounded_steering_tire_rotation_rate_radps =
        lateral->steering_tire_rotation_rate_radps;
    exact.limiter_reference_valid = lateral->limiter_reference_valid;
    exact.limiter_reference_steering_rad =
        lateral->limiter_reference_steering_rad;
    exact.command_dt_sec = steering_command_nominal_dt_sec_;
    exact.wheelbase_m = wheel_base_;
    exact.steering_output_gain = steering_tire_angle_gain_;
    exact.hard_steering_angle_limit_rad =
        free_run_live_exact_hard_steering_limit_rad_;
    exact.hard_steering_rate_limit_radps =
        free_run_live_exact_hard_steering_rate_limit_radps_;
    exact.steering_angle_limited = lateral->steering_angle_limited;
    exact.steering_rate_limited = lateral->steering_rate_limited;

    double required_spatial_horizon_m = lateral->lookahead_distance_m;
    if (std::isfinite(context->overtake_spatial_horizon_required_arc_m) &&
        context->overtake_spatial_horizon_required_arc_m > 0.0) {
      required_spatial_horizon_m =
          std::max(required_spatial_horizon_m,
                   context->overtake_spatial_horizon_required_arc_m);
    }
    const auto required_horizon_end =
        requiredHorizonEndIndex(*context->trajectory, context->nearest_index,
                                required_spatial_horizon_m);
    if (required_horizon_end.has_value()) {
      exact.required_horizon_end_trajectory_index =
          required_horizon_end.value();
    } else {
      exact.enabled = false;
    }
    if (exact.enabled) {
      pub_execution_envelope_->publish(makeControllerExecutionEnvelopeV2(
          command_envelope, input,
          plan_snapshot != nullptr ? plan_snapshot->plan.get() : nullptr,
          typed_plan_fresh, exact));
      return;
    }
  }
  pub_execution_envelope_->publish(makeControllerExecutionEnvelopeV1(
      command_envelope, input,
      plan_snapshot != nullptr ? plan_snapshot->plan.get() : nullptr,
      typed_plan_fresh));
}

void SimplePurePursuit::publishDebug(
    const rclcpp::Time &stamp, const Trajectory &control_trajectory,
    std::size_t nearest_traj_point_idx, double target_longitudinal_vel,
    double current_longitudinal_vel, double command_accel,
    double base_lookahead_distance, double desired_lookahead_distance,
    double lookahead_distance, double path_curvature,
    double signed_path_curvature, double curvature_window_distance,
    double lookahead_point_x, double lookahead_point_y, double rear_x,
    double rear_y, double alpha, double pure_pursuit_steering_tire_angle,
    double curvature_feedforward_steering_rad, double raw_steering_tire_angle,
    double steering_tire_angle, bool overtake_override_applied,
    double overtake_lateral_offset_m, double overtake_speed_cap_mps,
    double freshness_now_sec, bool mpc_horizon_applied,
    bool mpc_horizon_velocity_cap_applied, double mpc_horizon_velocity_cap_mps,
    std::int32_t evaluated_horizon_stamp_sec,
    std::uint32_t evaluated_horizon_stamp_nanosec,
    std::int32_t applied_horizon_stamp_sec,
    std::uint32_t applied_horizon_stamp_nanosec,
    const std::string &applied_horizon_source, int applied_horizon_mode_id,
    std::uint32_t applied_horizon_generation,
    const HorizonFreshnessResult &mpc_horizon_freshness,
    const std::string &trajectory_source,
    const std::string &overtake_override_apply_reason, bool v4_poc_contract,
    bool v4_poc_identity_required, bool v4_poc_identity_matched,
    bool v4_poc_geometry_applied, std::uint32_t v4_poc_generation,
    double overtake_spatial_horizon_arc_m,
    double overtake_spatial_horizon_required_arc_m,
    const ControlPosePrediction &control_pose) {
  if (debug_publish_period_sec_ <= 0.0 || !pub_debug_) {
    return;
  }

  if (!mpc_horizon_applied &&
      freshness_now_sec - last_debug_publish_sec_ < debug_publish_period_sec_) {
    return;
  }
  last_debug_publish_sec_ = freshness_now_sec;

  // rclcpp::Time does not expose ROS message fields directly. Preserve the
  // command timestamp exactly as it will appear on the wire in debug output.
  const builtin_interfaces::msg::Time stamp_msg = stamp;

  const auto &nearest = control_trajectory.points.at(nearest_traj_point_idx);
  const double ego_x = odometry_->pose.pose.position.x;
  const double ego_y = odometry_->pose.pose.position.y;
  const double ref_x = nearest.pose.position.x;
  const double ref_y = nearest.pose.position.y;
  const double ref_yaw = tf2::getYaw(nearest.pose.orientation);
  const double ego_yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  const double dx = ego_x - ref_x;
  const double dy = ego_y - ref_y;
  const double lateral_error_m =
      -std::sin(ref_yaw) * dx + std::cos(ref_yaw) * dy;
  const double yaw_error_rad =
      std::atan2(std::sin(ego_yaw - ref_yaw), std::cos(ego_yaw - ref_yaw));
  const double control_dx = control_pose.position.x - ref_x;
  const double control_dy = control_pose.position.y - ref_y;
  const double control_lateral_error_m =
      -std::sin(ref_yaw) * control_dx + std::cos(ref_yaw) * control_dy;
  const double control_yaw_error_rad =
      std::atan2(std::sin(control_pose.yaw - ref_yaw),
                 std::cos(control_pose.yaw - ref_yaw));
  const double odom_age_sec =
      inputAgeSec(last_odometry_receive_sec_, freshness_now_sec);
  const double trajectory_age_sec =
      inputAgeSec(last_trajectory_receive_sec_, freshness_now_sec);
  const double override_age_sec =
      overtake_override_active_
          ? inputAgeSec(last_overtake_override_sec_, freshness_now_sec)
          : -1.0;

  std::ostringstream json;
  json << "{"
       << "\"controller\":\"simple_pure_pursuit\","
       << "\"stale_input\":false,"
       << "\"odom_age_sec\":" << odom_age_sec << ","
       << "\"trajectory_age_sec\":" << trajectory_age_sec << ","
       << "\"override_age_sec\":" << override_age_sec << ","
       << "\"trajectory_source\":\"" << trajectory_source << "\","
       << "\"mpc_horizon_applied\":" << (mpc_horizon_applied ? "true" : "false")
       << ","
       << "\"mpc_horizon_velocity_cap_applied\":"
       << (mpc_horizon_velocity_cap_applied ? "true" : "false") << ","
       << "\"mpc_horizon_velocity_cap_mps\":" << mpc_horizon_velocity_cap_mps
       << ","
       << "\"command_stamp_sec\":" << stamp_msg.sec << ","
       << "\"command_stamp_nanosec\":" << stamp_msg.nanosec << ","
       << "\"evaluated_horizon_stamp_sec\":" << evaluated_horizon_stamp_sec
       << ","
       << "\"evaluated_horizon_stamp_nanosec\":"
       << evaluated_horizon_stamp_nanosec << ","
       << "\"applied_horizon_stamp_sec\":" << applied_horizon_stamp_sec << ","
       << "\"applied_horizon_stamp_nanosec\":" << applied_horizon_stamp_nanosec
       << ","
       << "\"applied_horizon_source\":\"" << applied_horizon_source << "\","
       << "\"applied_horizon_mode_id\":" << applied_horizon_mode_id << ","
       << "\"applied_horizon_generation\":" << applied_horizon_generation << ","
       << "\"mpc_horizon_age_sec\":" << mpc_horizon_freshness.age_sec << ","
       << "\"mpc_horizon_points\":" << mpc_horizon_freshness.point_count << ","
       << "\"mpc_horizon_start_distance_m\":"
       << mpc_horizon_freshness.start_distance_m << ","
       << "\"mpc_horizon_arc_length_m\":" << mpc_horizon_freshness.arc_length_m
       << ","
       << "\"mpc_health_required_for_horizon\":"
       << (require_solved_mpc_health_for_horizon_ ? "true" : "false") << ","
       << "\"mpc_health_status\":\"" << mpc_health_status_ << "\","
       << "\"mpc_health_age_sec\":" << mpcHealthAgeSec(freshness_now_sec) << ","
       << "\"mpc_infeasible_count\":" << mpc_health_infeasible_count_ << ","
       << "\"mpc_predicted_horizon_source\":\"" << mpc_predicted_horizon_source_
       << "\","
       << "\"mpc_horizon_contract_required\":"
       << (require_matching_overtake_horizon_contract_ ? "true" : "false")
       << ","
       << "\"mpc_horizon_contract_received\":"
       << (mpc_horizon_contract_received_ ? "true" : "false") << ","
       << "\"mpc_horizon_contract_age_sec\":"
       << inputAgeSec(last_mpc_predicted_horizon_contract_receive_sec_,
                      freshness_now_sec)
       << ","
       << "\"mpc_horizon_contract_source\":\"" << mpc_horizon_contract_source_
       << "\","
       << "\"mpc_horizon_contract_mode_id\":" << mpc_horizon_contract_mode_id_
       << ","
       << "\"mpc_horizon_contract_generation\":"
       << mpc_horizon_contract_generation_ << ","
       << "\"mpc_horizon_contract_solver_horizon_authorized\":"
       << (mpc_horizon_contract_solver_horizon_authorized_ ? "true" : "false")
       << ","
       << "\"mpc_horizon_contract_mandatory_lateral_avoidance\":"
       << (mpc_horizon_contract_mandatory_lateral_avoidance_ ? "true" : "false")
       << ","
       << "\"overtake_override_generation\":" << overtake_override_generation_
       << ","
       << "\"overtake_solver_horizon_authorized\":"
       << (overtake_solver_horizon_authorized_ ? "true" : "false") << ","
       << "\"overtake_mandatory_lateral_avoidance\":"
       << (overtake_mandatory_lateral_avoidance_ ? "true" : "false") << ","
       << "\"mpc_horizon_reject_reason\":\"" << mpc_horizon_freshness.reason
       << "\","
       << "\"nearest_trajectory_index\":" << nearest_traj_point_idx << ","
       << "\"target_speed_mps\":" << target_longitudinal_vel << ","
       << "\"current_speed_mps\":" << current_longitudinal_vel << ","
       << "\"command_accel_mps2\":" << command_accel << ","
       << "\"base_lookahead_distance_m\":" << base_lookahead_distance << ","
       << "\"desired_lookahead_distance_m\":" << desired_lookahead_distance
       << ","
       << "\"lookahead_distance_m\":" << lookahead_distance << ","
       << "\"path_curvature_1pm\":" << path_curvature << ","
       << "\"signed_path_curvature_1pm\":" << signed_path_curvature << ","
       << "\"curvature_window_distance_m\":" << curvature_window_distance << ","
       << "\"curvature_adaptive_lookahead_enabled\":"
       << (curvature_adaptive_lookahead_enabled_ ? "true" : "false") << ","
       << "\"lookahead_point_x\":" << lookahead_point_x << ","
       << "\"lookahead_point_y\":" << lookahead_point_y << ","
       << "\"rear_x\":" << rear_x << ","
       << "\"rear_y\":" << rear_y << ","
       << "\"alpha_rad\":" << alpha << ","
       << "\"pure_pursuit_steering_tire_angle_rad\":"
       << pure_pursuit_steering_tire_angle << ","
       << "\"curvature_feedforward_steering_rad\":"
       << curvature_feedforward_steering_rad << ","
       << "\"raw_steering_tire_angle_rad\":" << raw_steering_tire_angle << ","
       << "\"steering_tire_angle_rad\":" << steering_tire_angle << ","
       << "\"lateral_error_m\":" << lateral_error_m << ","
       << "\"yaw_error_rad\":" << yaw_error_rad << ","
       << "\"control_pose_x\":" << control_pose.position.x << ","
       << "\"control_pose_y\":" << control_pose.position.y << ","
       << "\"control_pose_yaw_rad\":" << control_pose.yaw << ","
       << "\"control_pose_shifted\":"
       << (control_pose.shifted ? "true" : "false") << ","
       << "\"control_pose_prediction_steps\":" << control_pose.prediction_steps
       << ","
       << "\"control_pose_lateral_error_m\":" << control_lateral_error_m << ","
       << "\"control_pose_yaw_error_rad\":" << control_yaw_error_rad << ","
       << "\"pp_control_delay_sec\":" << pp_control_delay_sec_ << ","
       << "\"pp_prediction_dt_sec\":" << pp_prediction_dt_sec_ << ","
       << "\"steering_source\":\"" << control_pose.steering_source << "\","
       << "\"steering_age_sec\":" << control_pose.steering_age_sec << ","
       << "\"current_steering_rad\":" << control_pose.current_steering_rad
       << ","
       << "\"predicted_applied_steering_rad\":"
       << control_pose.applied_steering_rad << ","
       << "\"use_external_target_vel\":"
       << (use_external_target_vel_ ? "true" : "false") << ","
       << "\"overtake_override_applied\":"
       << (overtake_override_applied ? "true" : "false") << ","
       << "\"overtake_override_apply_reason\":\""
       << overtake_override_apply_reason << "\","
       << "\"v4_poc_contract\":" << (v4_poc_contract ? "true" : "false") << ","
       << "\"v4_poc_identity_required\":"
       << (v4_poc_identity_required ? "true" : "false") << ","
       << "\"v4_poc_identity_matched\":"
       << (v4_poc_identity_matched ? "true" : "false") << ","
       << "\"v4_poc_geometry_applied\":"
       << (v4_poc_geometry_applied ? "true" : "false") << ","
       << "\"v4_poc_generation\":" << v4_poc_generation << ","
       << "\"overtake_spatial_horizon_arc_m\":"
       << (std::isfinite(overtake_spatial_horizon_arc_m)
               ? overtake_spatial_horizon_arc_m
               : -1.0)
       << ","
       << "\"overtake_spatial_horizon_required_arc_m\":"
       << (std::isfinite(overtake_spatial_horizon_required_arc_m)
               ? overtake_spatial_horizon_required_arc_m
               : -1.0)
       << ","
       << "\"overtake_mode_id\":" << overtake_mode_id_ << ","
       << "\"overtake_lateral_offset_m\":" << overtake_lateral_offset_m << ","
       << "\"overtake_speed_cap_mps\":" << overtake_speed_cap_mps << "}";

  String msg;
  msg.data = json.str();
  pub_debug_->publish(msg);
}
} // namespace simple_pure_pursuit

#ifndef SIMPLE_PURE_PURSUIT_NO_MAIN
int main(int argc, char const *argv[]) {
#ifdef SIMPLE_PURE_PURSUIT_TIMING_DIAGNOSTIC
  const char *probe_enabled_text =
      std::getenv("C002AY0_PP_CALLBACK_SPAN_ENABLED");
  const bool probe_enabled =
      probe_enabled_text != nullptr && std::string(probe_enabled_text) == "1";
  simple_pure_pursuit::timing_diagnostic::CallbackSpanRecorder::instance()
      .initialize(probe_enabled);
#endif
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
#ifdef SIMPLE_PURE_PURSUIT_TIMING_DIAGNOSTIC
  const bool diagnostic_written =
      simple_pure_pursuit::timing_diagnostic::CallbackSpanRecorder::instance()
          .writeFromEnvironment();
#endif
  rclcpp::shutdown();
#ifdef SIMPLE_PURE_PURSUIT_TIMING_DIAGNOSTIC
  return diagnostic_written ? 0 : 2;
#else
  return 0;
#endif
}
#endif
