#include "overtake_planner/pp_core_exact_snapshot.hpp"

#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <vector>

namespace overtake_planner {
namespace {

double stampSec(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

bool stampValid(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U &&
         (stamp.sec != 0 || stamp.nanosec != 0U);
}

template <typename Digest> bool digestPresent(const Digest &digest) {
  return std::any_of(digest.begin(), digest.end(),
                     [](std::uint8_t value) { return value != 0U; });
}

bool near(double left, double right, double tolerance = 1.0e-9) {
  return std::isfinite(left) && std::isfinite(right) &&
         std::abs(left - right) <= tolerance;
}

template <typename T>
void appendScalar(std::vector<std::uint8_t> &bytes, const T &value) {
  static_assert(std::is_trivially_copyable_v<T>);
  const auto *first = reinterpret_cast<const std::uint8_t *>(&value);
  bytes.insert(bytes.end(), first, first + sizeof(T));
}

std::array<std::uint8_t, 32U> boundedGeometryDigest(
    const multi_purpose_mpc_ros_msgs::msg::ControllerExecutionWitness
        &witness) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(128U + witness.applied_trajectory.points.size() * 96U);
  constexpr char domain[] = "PP_CORE_EXACT_BOUNDED_GEOMETRY_V1";
  bytes.insert(bytes.end(), domain, domain + sizeof(domain) - 1U);
  const auto &header = witness.applied_trajectory.header;
  const auto frame_size = static_cast<std::uint32_t>(header.frame_id.size());
  appendScalar(bytes, frame_size);
  bytes.insert(bytes.end(), header.frame_id.begin(), header.frame_id.end());
  appendScalar(bytes, header.stamp.sec);
  appendScalar(bytes, header.stamp.nanosec);
  appendScalar(bytes, witness.exact_geometry_original_point_count);
  appendScalar(bytes, witness.exact_geometry_first_source_index);
  appendScalar(bytes, witness.exact_geometry_last_source_index);
  for (const auto &point : witness.applied_trajectory.points) {
    appendScalar(bytes, point.pose.position.x);
    appendScalar(bytes, point.pose.position.y);
    appendScalar(bytes, point.pose.position.z);
    appendScalar(bytes, point.pose.orientation.x);
    appendScalar(bytes, point.pose.orientation.y);
    appendScalar(bytes, point.pose.orientation.z);
    appendScalar(bytes, point.pose.orientation.w);
    appendScalar(bytes, point.longitudinal_velocity_mps);
    appendScalar(bytes, point.lateral_velocity_mps);
    appendScalar(bytes, point.acceleration_mps2);
    appendScalar(bytes, point.heading_rate_rps);
  }
  return overtake_transport_contract::c002ay0::sha256(bytes);
}

} // namespace

PurePursuitExactSnapshot validatePurePursuitExactSnapshotV2(
    const multi_purpose_mpc_ros_msgs::msg::ControllerExecutionEnvelope
        &envelope,
    const PurePursuitCandidateBinding &expected, double receive_time_sec,
    double future_tolerance_sec) {
  PurePursuitExactSnapshot result;
  const auto fail = [&result](const char *reason) {
    result.valid = false;
    result.reason = reason;
    return result;
  };
  const auto &command = envelope.command_envelope;
  const auto &witness = envelope.witness;
  const auto &key = witness.plan_sample_key;
  if (envelope.schema_version != 2U || witness.schema_version != 2U ||
      command.schema_version != 2U || !witness.diagnostic_only ||
      !witness.exact_snapshot_complete || witness.authority_eligible) {
    return fail("schema_or_diagnostic_boundary_invalid");
  }
  if (envelope.producer_instance_id == 0U || envelope.command_sequence == 0U ||
      envelope.producer_instance_id != command.producer_instance_id ||
      envelope.command_sequence != command.command_sequence ||
      envelope.plan_generation != command.plan_generation ||
      envelope.header != command.header ||
      command.command.stamp != envelope.header.stamp ||
      command.command.lateral.stamp != envelope.header.stamp ||
      command.command.longitudinal.stamp != envelope.header.stamp ||
      witness.control_pose_stamp != envelope.header.stamp) {
    return fail("atomic_command_binding_invalid");
  }
  if (!stampValid(envelope.header.stamp) || !stampValid(key.plan_stamp) ||
      key.race_arm_epoch != expected.race_arm_epoch ||
      key.planner_instance_id != expected.planner_instance_id ||
      key.attempt_id != expected.attempt_id ||
      key.target_vehicle_id != expected.target_vehicle_id ||
      key.pass_direction != expected.pass_direction ||
      key.connector_transaction_id != expected.connector_transaction_id ||
      key.plan_generation != expected.plan_generation ||
      !near(stampSec(key.plan_stamp), expected.plan_stamp_sec) ||
      witness.candidate_revision != expected.candidate_revision ||
      witness.candidate_content_sha256 != expected.candidate_content_sha256 ||
      witness.plan_sample_key != command.plan_sample_key ||
      witness.candidate_revision != command.candidate_revision ||
      witness.candidate_content_sha256 != command.candidate_content_sha256) {
    return fail("current_candidate_binding_mismatch");
  }
  const double command_stamp_sec = stampSec(envelope.header.stamp);
  const double lease_duration_sec = witness.diagnostic_lease_duration_sec;
  if (!std::isfinite(receive_time_sec) ||
      !std::isfinite(future_tolerance_sec) || future_tolerance_sec < 0.0 ||
      !std::isfinite(lease_duration_sec) || lease_duration_sec <= 0.0 ||
      lease_duration_sec > 0.1 ||
      command_stamp_sec > receive_time_sec + future_tolerance_sec) {
    return fail("command_stamp_future_or_invalid");
  }
  if (receive_time_sec > command_stamp_sec + lease_duration_sec) {
    return fail("diagnostic_lease_expired");
  }
  if (!digestPresent(witness.base_source_sha256) ||
      !digestPresent(witness.controller_implementation_sha256) ||
      !digestPresent(witness.controller_config_sha256) ||
      !digestPresent(witness.candidate_content_sha256) ||
      witness.base_source_generation == 0U ||
      witness.base_source_point_count == 0U ||
      !stampValid(witness.base_source_stamp)) {
    return fail("source_digest_binding_invalid");
  }
  const auto original_count = witness.exact_geometry_original_point_count;
  const auto first = witness.exact_geometry_first_source_index;
  const auto last = witness.exact_geometry_last_source_index;
  const auto nearest = witness.nearest_trajectory_index;
  const auto selected = witness.selected_lookahead_trajectory_index;
  if (original_count == 0U ||
      original_count != witness.base_source_point_count || first != nearest ||
      first > selected || selected > last || last >= original_count ||
      witness.applied_trajectory.points.empty() ||
      witness.applied_trajectory.points.size() > 100U ||
      witness.applied_trajectory.points.size() != last - first + 1U ||
      witness.applied_trajectory.header.frame_id.empty() ||
      witness.applied_trajectory.header.stamp != witness.source_stamp) {
    return fail("bounded_geometry_invalid");
  }
  for (const auto &point : witness.applied_trajectory.points) {
    if (!std::isfinite(point.pose.position.x) ||
        !std::isfinite(point.pose.position.y) ||
        !std::isfinite(point.longitudinal_velocity_mps)) {
      return fail("bounded_geometry_nonfinite");
    }
  }
  const auto finite = [](std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
  };
  if (!finite({witness.control_speed_mps, witness.resolved_lookahead_distance_m,
               witness.geometric_steering_tire_angle_rad,
               witness.curvature_feedforward_steering_rad,
               witness.raw_exact_steering_tire_angle_rad,
               witness.requested_output_steering_tire_angle_rad,
               witness.bounded_exact_steering_tire_angle_rad,
               witness.requested_steering_tire_rotation_rate_radps,
               witness.bounded_exact_steering_tire_rotation_rate_radps,
               witness.limiter_reference_steering_rad, witness.command_dt_sec,
               witness.wheelbase_m, witness.steering_output_gain,
               witness.hard_steering_angle_limit_rad,
               witness.hard_steering_rate_limit_radps,
               witness.diagnostic_lease_duration_sec}) ||
      !witness.limiter_reference_valid || witness.control_speed_mps < 0.0 ||
      witness.resolved_lookahead_distance_m <= 0.0 ||
      witness.command_dt_sec <= 0.0 || witness.wheelbase_m <= 0.0 ||
      witness.hard_steering_angle_limit_rad <= 0.0 ||
      witness.hard_steering_rate_limit_radps <= 0.0 ||
      witness.diagnostic_lease_duration_sec <= 0.0 ||
      witness.diagnostic_lease_duration_sec > 0.1) {
    return fail("exact_scalar_invalid");
  }
  const double raw_expected = witness.geometric_steering_tire_angle_rad +
                              witness.curvature_feedforward_steering_rad;
  const double requested_expected = witness.steering_output_gain * raw_expected;
  const double bounded_reference =
      std::clamp(witness.limiter_reference_steering_rad,
                 -witness.hard_steering_angle_limit_rad,
                 witness.hard_steering_angle_limit_rad);
  const double angle_clamped_request =
      std::clamp(requested_expected, -witness.hard_steering_angle_limit_rad,
                 witness.hard_steering_angle_limit_rad);
  const double max_delta_rad =
      witness.hard_steering_rate_limit_radps * witness.command_dt_sec;
  if (!std::isfinite(max_delta_rad) || max_delta_rad <= 0.0) {
    return fail("exact_scalar_relation_invalid");
  }
  const double rate_clamped_request =
      std::clamp(angle_clamped_request,
                 bounded_reference - max_delta_rad,
                 bounded_reference + max_delta_rad);
  const double bounded_angle_expected =
      std::clamp(rate_clamped_request,
                 -witness.hard_steering_angle_limit_rad,
                 witness.hard_steering_angle_limit_rad);
  const double requested_rate_expected =
      (requested_expected - bounded_reference) /
      witness.command_dt_sec;
  const double bounded_rate_expected =
      (bounded_angle_expected - bounded_reference) / witness.command_dt_sec;
  const bool angle_limited_expected =
      std::abs(requested_expected - angle_clamped_request) > 1.0e-12;
  const bool rate_limited_expected =
      std::abs(angle_clamped_request - bounded_angle_expected) > 1.0e-12;
  if (!near(witness.raw_exact_steering_tire_angle_rad, raw_expected) ||
      !near(witness.requested_output_steering_tire_angle_rad,
            requested_expected) ||
      !near(witness.requested_steering_tire_rotation_rate_radps,
            requested_rate_expected) ||
      !near(witness.bounded_exact_steering_tire_angle_rad,
            bounded_angle_expected) ||
      !near(witness.bounded_exact_steering_tire_rotation_rate_radps,
            bounded_rate_expected) ||
      witness.steering_angle_limited != angle_limited_expected ||
      witness.steering_rate_limited != rate_limited_expected) {
    return fail("exact_scalar_relation_invalid");
  }
  // The current PP wire contract deliberately carries zero rotation_rate.
  // The applied rate limit is proven independently above by the bounded angle
  // delta, rather than by treating this wire placeholder as the limiter rate.
  if (!near(command.command.lateral.steering_tire_angle,
            bounded_angle_expected, 1.0e-6) ||
      !near(command.command.lateral.steering_tire_rotation_rate, 0.0,
            1.0e-6)) {
    return fail("command_wire_contract_invalid");
  }

  result.valid = true;
  result.reason = "complete";
  result.producer_instance_id = envelope.producer_instance_id;
  result.command_sequence = envelope.command_sequence;
  result.command_stamp_sec = command_stamp_sec;
  result.valid_until_sec =
      result.command_stamp_sec + witness.diagnostic_lease_duration_sec;
  result.bounded_geometry_sha256 = boundedGeometryDigest(witness);
  result.binding = expected;
  result.evaluator_input.ego.valid = true;
  result.evaluator_input.ego.stamp_sec = result.command_stamp_sec;
  result.evaluator_input.ego.x = witness.control_pose.position.x;
  result.evaluator_input.ego.y = witness.control_pose.position.y;
  const auto &q = witness.control_pose.orientation;
  result.evaluator_input.ego.yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  result.evaluator_input.ego.v = witness.control_speed_mps;
  result.evaluator_input.active_lookahead_valid = true;
  result.evaluator_input.active_lookahead_distance_m =
      witness.resolved_lookahead_distance_m;
  result.evaluator_input.nearest_source_index_valid = true;
  result.evaluator_input.nearest_source_index = nearest;
  result.evaluator_input.curvature_feedforward_valid = true;
  result.evaluator_input.curvature_feedforward_steering_rad =
      witness.curvature_feedforward_steering_rad;
  result.evaluator_input.steering_reference_valid = true;
  result.evaluator_input.steering_reference_angle_rad =
      witness.limiter_reference_steering_rad;
  result.evaluator_input.steering_command_dt_sec = witness.command_dt_sec;
  result.selected_lookahead_source_index = selected;
  result.lookahead_endpoint_fallback = witness.lookahead_endpoint_fallback;
  result.geometric_steering_tire_angle_rad =
      witness.geometric_steering_tire_angle_rad;
  result.raw_steering_tire_angle_rad =
      witness.raw_exact_steering_tire_angle_rad;
  result.requested_output_steering_tire_angle_rad =
      witness.requested_output_steering_tire_angle_rad;
  result.bounded_steering_tire_angle_rad =
      witness.bounded_exact_steering_tire_angle_rad;
  result.requested_steering_rate_radps =
      witness.requested_steering_tire_rotation_rate_radps;
  result.bounded_steering_rate_radps =
      witness.bounded_exact_steering_tire_rotation_rate_radps;
  result.steering_angle_limited = witness.steering_angle_limited;
  result.steering_rate_limited = witness.steering_rate_limited;
  return result;
}

} // namespace overtake_planner
