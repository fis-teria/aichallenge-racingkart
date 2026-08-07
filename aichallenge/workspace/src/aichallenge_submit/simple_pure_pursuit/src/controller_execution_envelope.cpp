#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>

namespace simple_pure_pursuit {
namespace {

bool executionTrajectoryValuesValid(const Trajectory &trajectory) {
  for (const auto &point : trajectory.points) {
    const auto &position = point.pose.position;
    const auto &orientation = point.pose.orientation;
    const double orientation_norm =
        std::hypot(std::hypot(orientation.x, orientation.y),
                   std::hypot(orientation.z, orientation.w));
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w) || !std::isfinite(orientation_norm) ||
        orientation_norm <= 1.0e-9 ||
        !std::isfinite(point.longitudinal_velocity_mps) ||
        point.longitudinal_velocity_mps < 0.0F) {
      return false;
    }
  }
  return true;
}

bool stampValid(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
}

bool poseFinite(const Pose &pose) {
  const double orientation_norm =
      std::hypot(std::hypot(pose.orientation.x, pose.orientation.y),
                 std::hypot(pose.orientation.z, pose.orientation.w));
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && std::isfinite(orientation_norm) &&
         orientation_norm > 1.0e-9;
}

template <typename Digest> bool digestPresent(const Digest &digest) {
  return std::any_of(digest.begin(), digest.end(),
                     [](std::uint8_t value) { return value != 0U; });
}

std::string diagnosticFingerprint(const std::vector<std::uint8_t> &bytes) {
  std::uint64_t fingerprint = 1469598103934665603ULL;
  for (const std::uint8_t byte : bytes) {
    fingerprint ^= byte;
    fingerprint *= 1099511628211ULL;
  }
  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(16) << fingerprint;
  return stream.str();
}

} // namespace

CanonicalSourcePayload
canonicalizeSourcePayload(const Float32MultiArray &source) {
  CanonicalSourcePayload result;
  result.bytes.reserve(kMaxExecutionSourcePayloadBytes);

  const auto append_byte = [&result](std::uint8_t byte) {
    if (result.original_size_bytes < kMaxExecutionSourcePayloadBytes) {
      result.bytes.push_back(byte);
    }
    if (result.original_size_bytes <
        std::numeric_limits<std::uint64_t>::max()) {
      ++result.original_size_bytes;
    }
  };
  const auto append_u32 = [&append_byte](std::uint32_t value) {
    for (unsigned int shift = 0U; shift < 32U; shift += 8U) {
      append_byte(static_cast<std::uint8_t>((value >> shift) & 0xffU));
    }
  };
  const auto append_string = [&append_byte,
                              &append_u32](const std::string &value) {
    append_u32(static_cast<std::uint32_t>(std::min<std::size_t>(
        value.size(), std::numeric_limits<std::uint32_t>::max())));
    for (const unsigned char byte : value) {
      append_byte(byte);
    }
  };

  append_byte('F');
  append_byte('3');
  append_byte('2');
  append_byte('A');
  append_u32(source.layout.data_offset);
  append_u32(static_cast<std::uint32_t>(source.layout.dim.size()));
  for (const auto &dimension : source.layout.dim) {
    append_string(dimension.label);
    append_u32(dimension.size);
    append_u32(dimension.stride);
  }
  append_u32(static_cast<std::uint32_t>(source.data.size()));
  for (const float value : source.data) {
    std::uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    append_u32(bits);
  }
  result.complete =
      result.original_size_bytes <= kMaxExecutionSourcePayloadBytes;
  if (!result.complete) {
    result.bytes.resize(kMaxExecutionSourcePayloadBytes);
  }
  return result;
}

bool typedPlanOrderAcceptable(
    const builtin_interfaces::msg::Time &candidate_stamp,
    std::uint32_t candidate_generation,
    const builtin_interfaces::msg::Time &now_stamp, bool has_previous,
    const builtin_interfaces::msg::Time &previous_stamp,
    std::uint32_t previous_generation) {
  if (!stampValid(candidate_stamp) || !stampValid(now_stamp)) {
    return false;
  }
  const auto later_than = [](const builtin_interfaces::msg::Time &left,
                             const builtin_interfaces::msg::Time &right) {
    return left.sec > right.sec ||
           (left.sec == right.sec && left.nanosec > right.nanosec);
  };
  if (later_than(candidate_stamp, now_stamp)) {
    return false;
  }
  if (!has_previous) {
    return true;
  }
  return candidate_generation >= previous_generation &&
         !later_than(previous_stamp, candidate_stamp);
}

ControllerExecutionEnvelope makeControllerExecutionEnvelopeV1(
    const ControllerCommandEnvelope &command_envelope,
    const ControllerExecutionWitnessInput &input,
    const OvertakePlan *typed_plan, bool typed_plan_fresh) {
  ControllerExecutionEnvelope envelope;
  envelope.header = command_envelope.header;
  envelope.schema_version = 1U;
  envelope.producer_instance_id = command_envelope.producer_instance_id;
  envelope.command_sequence = command_envelope.command_sequence;
  envelope.plan_generation = command_envelope.plan_generation;
  envelope.command_envelope = command_envelope;

  auto &witness = envelope.witness;
  witness.schema_version = 1U;
  witness.controller_role = input.controller_role;
  witness.trajectory_source = input.trajectory_source;
  witness.identity_source = ControllerExecutionWitness::IDENTITY_NONE;
  witness.authority_eligible = false;
  witness.source_generation = input.source_generation;
  witness.source_binding_complete =
      input.trajectory_source ==
          ControllerExecutionWitness::SOURCE_REFERENCE_OVERRIDE &&
      input.source_generation == command_envelope.plan_generation &&
      input.source_generation > 0U && input.source_payload.complete &&
      !input.source_payload.bytes.empty() &&
      input.source_payload.bytes.size() <= kMaxExecutionSourcePayloadBytes;
  witness.source_payload_original_size_bytes =
      input.source_payload.original_size_bytes;
  if (witness.source_binding_complete) {
    witness.source_payload_fingerprint =
        diagnosticFingerprint(input.source_payload.bytes);
  }

  const bool typed_plan_header_valid =
      typed_plan != nullptr && stampValid(typed_plan->header.stamp) &&
      !typed_plan->header.frame_id.empty() &&
      typed_plan->header == typed_plan->trajectory.header &&
      typed_plan->header.frame_id == input.applied_trajectory.header.frame_id;
  const bool typed_identity_exact =
      typed_plan != nullptr && typed_plan_fresh &&
      typed_plan->trajectory_authorized && typed_plan_header_valid &&
      typed_plan->plan_generation == command_envelope.plan_generation &&
      typed_plan->attempt_id > 0U && !typed_plan->target_vehicle_id.empty() &&
      (typed_plan->pass_direction == -1 || typed_plan->pass_direction == 1);
  if (typed_identity_exact) {
    witness.identity_source =
        ControllerExecutionWitness::IDENTITY_TYPED_OVERTAKE_PLAN;
    witness.plan_stamp = typed_plan->header.stamp;
    witness.attempt_id = typed_plan->attempt_id;
    witness.target_vehicle_id = typed_plan->target_vehicle_id;
    witness.pass_direction = typed_plan->pass_direction;
  }
  witness.typed_plan_geometry_matches_applied =
      typed_identity_exact &&
      typed_plan->trajectory == input.applied_trajectory;

  witness.source_stamp = input.source_stamp;
  witness.reference_stamp = input.reference_stamp;
  witness.base_trajectory = input.base_trajectory;
  witness.applied_trajectory = input.applied_trajectory;
  if (witness.base_trajectory.points.size() > kMaxExecutionTrajectoryPoints) {
    witness.base_trajectory.points.resize(kMaxExecutionTrajectoryPoints);
  }
  if (witness.applied_trajectory.points.size() >
      kMaxExecutionTrajectoryPoints) {
    witness.applied_trajectory.points.resize(kMaxExecutionTrajectoryPoints);
  }
  witness.nearest_trajectory_index = static_cast<std::uint32_t>(std::min(
      input.nearest_trajectory_index,
      static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  witness.control_pose = input.control_pose;
  witness.trajectory_progress_m =
      static_cast<float>(input.trajectory_progress_m);
  witness.available_spatial_horizon_m =
      static_cast<float>(input.available_spatial_horizon_m);
  witness.required_spatial_horizon_m =
      static_cast<float>(input.required_spatial_horizon_m);
  witness.raw_steering_tire_angle_rad =
      static_cast<float>(input.raw_steering_tire_angle_rad);
  witness.bounded_steering_tire_angle_rad =
      static_cast<float>(input.bounded_steering_tire_angle_rad);
  witness.raw_steering_tire_rotation_rate_radps =
      static_cast<float>(input.raw_steering_tire_rotation_rate_radps);
  witness.bounded_steering_tire_rotation_rate_radps =
      static_cast<float>(input.bounded_steering_tire_rotation_rate_radps);
  witness.steering_tire_angle_limit_rad =
      static_cast<float>(input.steering_tire_angle_limit_rad);
  witness.steering_tire_rotation_rate_limit_radps =
      static_cast<float>(input.steering_tire_rotation_rate_limit_radps);
  const auto rollout_end =
      input.rollout_samples.begin() +
      static_cast<std::ptrdiff_t>(
          std::min(input.rollout_samples.size(), kMaxExecutionRolloutSamples));
  witness.rollout_samples.assign(input.rollout_samples.begin(), rollout_end);

  const auto fail = [&witness](const char *reason) {
    witness.shadow_geometry_complete = false;
    witness.incompleteness_reason = reason;
  };

  if (input.controller_role != ControllerExecutionWitness::ROLE_PRIMARY &&
      input.controller_role != ControllerExecutionWitness::ROLE_RECOVERY) {
    fail("controller_role_unknown");
  } else if (input.trajectory_source <
                 ControllerExecutionWitness::SOURCE_REFERENCE_TRAJECTORY ||
             input.trajectory_source >
                 ControllerExecutionWitness::SOURCE_PREDICTED_HORIZON) {
    fail("trajectory_source_unknown");
  } else if (input.header != command_envelope.header ||
             command_envelope.schema_version != 1U ||
             command_envelope.producer_instance_id == 0U ||
             command_envelope.command_sequence == 0U ||
             command_envelope.command.stamp != command_envelope.header.stamp ||
             command_envelope.command.longitudinal.stamp !=
                 command_envelope.header.stamp ||
             command_envelope.command.lateral.stamp !=
                 command_envelope.header.stamp ||
             !std::isfinite(command_envelope.command.longitudinal.speed) ||
             !std::isfinite(
                 command_envelope.command.longitudinal.acceleration) ||
             !std::isfinite(command_envelope.command.longitudinal.jerk) ||
             !std::isfinite(
                 command_envelope.command.lateral.steering_tire_angle) ||
             !std::isfinite(command_envelope.command.lateral
                                .steering_tire_rotation_rate)) {
    fail("command_envelope_binding_invalid");
  } else if (typed_plan == nullptr) {
    fail("typed_plan_missing");
  } else if (!typed_plan_fresh) {
    fail("typed_plan_stale_or_out_of_order");
  } else if (!typed_plan->trajectory_authorized) {
    fail("typed_plan_trajectory_unauthorized");
  } else if (!typed_plan_header_valid) {
    fail("typed_plan_header_or_frame_invalid");
  } else if (typed_plan->plan_generation != command_envelope.plan_generation) {
    fail("typed_plan_generation_mismatch");
  } else if (typed_plan->attempt_id == 0U ||
             typed_plan->target_vehicle_id.empty() ||
             (typed_plan->pass_direction != -1 &&
              typed_plan->pass_direction != 1)) {
    fail("typed_plan_identity_invalid");
  } else if (!witness.typed_plan_geometry_matches_applied) {
    fail("typed_plan_geometry_relation_unproven");
  } else if (!stampValid(input.source_stamp) ||
             !stampValid(input.reference_stamp) ||
             input.base_trajectory.header.frame_id.empty() ||
             input.applied_trajectory.header.frame_id.empty() ||
             input.base_trajectory.header.frame_id !=
                 input.applied_trajectory.header.frame_id ||
             input.base_trajectory.header.stamp != input.reference_stamp ||
             input.applied_trajectory.header.stamp != input.source_stamp) {
    fail("source_reference_frame_or_stamp_invalid");
  } else if (!witness.source_binding_complete) {
    fail("source_binding_incomplete");
  } else if (input.base_trajectory.points.empty() ||
             input.applied_trajectory.points.empty()) {
    fail("trajectory_empty");
  } else if (input.base_trajectory.points.size() >
                 kMaxExecutionTrajectoryPoints ||
             input.applied_trajectory.points.size() >
                 kMaxExecutionTrajectoryPoints) {
    fail("trajectory_point_limit_exceeded");
  } else if (!executionTrajectoryValuesValid(input.base_trajectory) ||
             !executionTrajectoryValuesValid(input.applied_trajectory) ||
             !poseFinite(input.control_pose) ||
             input.nearest_trajectory_index >=
                 input.applied_trajectory.points.size()) {
    fail("trajectory_geometry_invalid");
  } else if (!std::isfinite(input.trajectory_progress_m) ||
             input.trajectory_progress_m < 0.0 ||
             !std::isfinite(input.available_spatial_horizon_m) ||
             !std::isfinite(input.required_spatial_horizon_m) ||
             input.available_spatial_horizon_m < 0.0 ||
             input.required_spatial_horizon_m <= 0.0 ||
             input.available_spatial_horizon_m <
                 input.required_spatial_horizon_m) {
    fail("spatial_horizon_invalid");
  } else if (!std::isfinite(input.raw_steering_tire_angle_rad) ||
             !std::isfinite(input.bounded_steering_tire_angle_rad) ||
             !std::isfinite(input.raw_steering_tire_rotation_rate_radps) ||
             !std::isfinite(input.bounded_steering_tire_rotation_rate_radps) ||
             !std::isfinite(input.steering_tire_angle_limit_rad) ||
             input.steering_tire_angle_limit_rad <= 0.0 ||
             !std::isfinite(input.steering_tire_rotation_rate_limit_radps) ||
             input.steering_tire_rotation_rate_limit_radps <= 0.0 ||
             std::abs(input.raw_steering_tire_angle_rad) >
                 input.steering_tire_angle_limit_rad ||
             std::abs(input.bounded_steering_tire_angle_rad) >
                 input.steering_tire_angle_limit_rad ||
             std::abs(input.raw_steering_tire_rotation_rate_radps) >
                 input.steering_tire_rotation_rate_limit_radps ||
             std::abs(input.bounded_steering_tire_rotation_rate_radps) >
                 input.steering_tire_rotation_rate_limit_radps) {
    fail("steering_bounds_incomplete");
  } else if (input.rollout_samples.empty()) {
    fail("rollout_unavailable");
  } else if (input.rollout_samples.size() > kMaxExecutionRolloutSamples) {
    fail("rollout_sample_limit_exceeded");
  } else {
    bool rollout_valid = true;
    float previous_time_sec = -1.0F;
    float previous_progress_m = static_cast<float>(input.trajectory_progress_m);
    for (std::size_t i = 0U; i < input.rollout_samples.size(); ++i) {
      const auto &sample = input.rollout_samples[i];
      const bool first_sample_matches =
          i != 0U ||
          (std::abs(sample.elapsed_time_sec) <= 1.0e-6F &&
           std::abs(sample.progress_m -
                    static_cast<float>(input.trajectory_progress_m)) <=
               1.0e-5F &&
           sample.pose == input.control_pose);
      if (!std::isfinite(sample.elapsed_time_sec) ||
          !std::isfinite(sample.progress_m) ||
          !std::isfinite(sample.speed_mps) ||
          !std::isfinite(sample.raw_steering_tire_angle_rad) ||
          !std::isfinite(sample.bounded_steering_tire_angle_rad) ||
          !std::isfinite(sample.raw_steering_tire_rotation_rate_radps) ||
          !std::isfinite(sample.bounded_steering_tire_rotation_rate_radps) ||
          !poseFinite(sample.pose) || !first_sample_matches ||
          sample.elapsed_time_sec < previous_time_sec ||
          sample.progress_m < previous_progress_m ||
          std::abs(sample.raw_steering_tire_angle_rad) >
              input.steering_tire_angle_limit_rad ||
          std::abs(sample.bounded_steering_tire_angle_rad) >
              input.steering_tire_angle_limit_rad ||
          std::abs(sample.raw_steering_tire_rotation_rate_radps) >
              input.steering_tire_rotation_rate_limit_radps ||
          std::abs(sample.bounded_steering_tire_rotation_rate_radps) >
              input.steering_tire_rotation_rate_limit_radps) {
        rollout_valid = false;
        break;
      }
      previous_time_sec = sample.elapsed_time_sec;
      previous_progress_m = sample.progress_m;
    }
    const double rollout_horizon_m =
        static_cast<double>(input.rollout_samples.back().progress_m) -
        input.trajectory_progress_m;
    if (!rollout_valid || !std::isfinite(rollout_horizon_m) ||
        rollout_horizon_m + 1.0e-6 < input.required_spatial_horizon_m) {
      fail("rollout_geometry_incomplete");
    }
  }

  if (witness.incompleteness_reason.empty()) {
    witness.shadow_geometry_complete = true;
    witness.incompleteness_reason = "complete";
  }
  // authority_eligible remains false for every AW1 sample, including complete
  // PRIMARY geometry. No consumer is allowed to infer motion authority here.
  return envelope;
}

ControllerExecutionEnvelope makeControllerExecutionEnvelopeV2(
    const ControllerCommandEnvelope &command_envelope,
    const ControllerExecutionWitnessInput &input,
    const OvertakePlan *typed_plan, bool typed_plan_fresh,
    const PurePursuitExactSnapshotInput &exact) {
  auto envelope = makeControllerExecutionEnvelopeV1(
      command_envelope, input, typed_plan, typed_plan_fresh);
  auto &witness = envelope.witness;
  witness.diagnostic_only = true;
  witness.exact_snapshot_complete = false;
  witness.exact_snapshot_reason = "exact_snapshot_disabled";
  witness.authority_eligible = false;
  if (!exact.enabled) {
    return envelope;
  }

  envelope.schema_version = 2U;
  witness.schema_version = 2U;
  witness.exact_snapshot_reason = "exact_snapshot_invalid";
  witness.plan_sample_key = command_envelope.plan_sample_key;
  witness.candidate_revision = command_envelope.candidate_revision;
  witness.candidate_content_sha256 = command_envelope.candidate_content_sha256;
  witness.base_source_stamp = exact.base_source_key.source_stamp;
  witness.base_source_generation = exact.base_source_key.source_generation;
  witness.base_source_point_count = exact.base_source_key.original_point_count;
  witness.base_source_sha256 = exact.base_source_key.baseline_reference_sha256;
  witness.controller_implementation_sha256 =
      exact.base_source_key.controller_implementation_sha256;
  witness.controller_config_sha256 =
      exact.base_source_key.controller_config_sha256;
  witness.control_pose_stamp = exact.control_pose_stamp;
  witness.diagnostic_lease_duration_sec =
      static_cast<float>(exact.diagnostic_lease_duration_sec);
  witness.exact_geometry_original_point_count =
      static_cast<std::uint32_t>(std::min(
          input.applied_trajectory.points.size(),
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  witness.exact_geometry_first_source_index =
      static_cast<std::uint32_t>(std::min(
          input.nearest_trajectory_index,
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  witness.exact_geometry_last_source_index =
      static_cast<std::uint32_t>(std::min(
          exact.required_horizon_end_trajectory_index,
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  witness.selected_lookahead_trajectory_index =
      static_cast<std::uint32_t>(std::min(
          exact.selected_lookahead_trajectory_index,
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  witness.lookahead_endpoint_fallback = exact.lookahead_endpoint_fallback;
  witness.control_speed_mps = exact.control_speed_mps;
  witness.resolved_lookahead_distance_m = exact.resolved_lookahead_distance_m;
  witness.geometric_steering_tire_angle_rad =
      exact.geometric_steering_tire_angle_rad;
  witness.curvature_feedforward_steering_rad =
      exact.curvature_feedforward_steering_rad;
  witness.raw_exact_steering_tire_angle_rad = exact.raw_steering_tire_angle_rad;
  witness.requested_output_steering_tire_angle_rad =
      exact.requested_output_steering_tire_angle_rad;
  witness.bounded_exact_steering_tire_angle_rad =
      exact.bounded_steering_tire_angle_rad;
  witness.requested_steering_tire_rotation_rate_radps =
      exact.requested_steering_tire_rotation_rate_radps;
  witness.bounded_exact_steering_tire_rotation_rate_radps =
      exact.bounded_steering_tire_rotation_rate_radps;
  witness.limiter_reference_valid = exact.limiter_reference_valid;
  witness.limiter_reference_steering_rad = exact.limiter_reference_steering_rad;
  witness.command_dt_sec = exact.command_dt_sec;
  witness.wheelbase_m = exact.wheelbase_m;
  witness.steering_output_gain = exact.steering_output_gain;
  witness.hard_steering_angle_limit_rad = exact.hard_steering_angle_limit_rad;
  witness.hard_steering_rate_limit_radps = exact.hard_steering_rate_limit_radps;
  witness.steering_angle_limited = exact.steering_angle_limited;
  witness.steering_rate_limited = exact.steering_rate_limited;

  const auto all_finite = [](std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
  };
  const bool typed_plan_binding_valid =
      typed_plan != nullptr && typed_plan_fresh &&
      typed_plan->trajectory_authorized &&
      typed_plan->plan_generation == command_envelope.plan_generation &&
      typed_plan->candidate_revision == command_envelope.candidate_revision &&
      typed_plan->candidate_content_sha256 ==
          command_envelope.candidate_content_sha256;
  if (command_envelope.schema_version != 2U ||
      command_envelope.producer_instance_id == 0U ||
      command_envelope.command_sequence == 0U || !typed_plan_binding_valid ||
      !digestPresent(command_envelope.candidate_content_sha256)) {
    witness.exact_snapshot_reason = "candidate_binding_invalid";
  } else if (!exact.base_source_binding_valid ||
             exact.base_source_key.controller_instance_id !=
                 command_envelope.producer_instance_id ||
             exact.base_source_key.source_generation == 0U ||
             exact.base_source_key.original_point_count == 0U ||
             !stampValid(exact.base_source_key.source_stamp) ||
             exact.base_source_key.source_stamp != input.reference_stamp ||
             exact.base_source_key.original_point_count !=
                 input.base_trajectory.points.size() ||
             !digestPresent(exact.base_source_key.baseline_reference_sha256) ||
             !digestPresent(
                 exact.base_source_key.controller_implementation_sha256) ||
             !digestPresent(exact.base_source_key.controller_config_sha256)) {
    witness.exact_snapshot_reason = "base_source_binding_invalid";
  } else if (input.trajectory_source !=
                 ControllerExecutionWitness::SOURCE_REFERENCE_OVERRIDE ||
             input.applied_trajectory.points.empty() ||
             input.nearest_trajectory_index >=
                 input.applied_trajectory.points.size() ||
             exact.selected_lookahead_trajectory_index <
                 input.nearest_trajectory_index ||
             exact.selected_lookahead_trajectory_index >=
                 input.applied_trajectory.points.size()) {
    witness.exact_snapshot_reason = "applied_geometry_binding_invalid";
  } else if (exact.required_horizon_end_trajectory_index <
                 exact.selected_lookahead_trajectory_index ||
             exact.required_horizon_end_trajectory_index >=
                 input.applied_trajectory.points.size() ||
             exact.required_horizon_end_trajectory_index -
                     input.nearest_trajectory_index + 1U >
                 kMaxExecutionTrajectoryPoints) {
    witness.exact_snapshot_reason = "bounded_geometry_invalid";
  } else if (!stampValid(exact.control_pose_stamp) ||
             exact.control_pose_stamp != command_envelope.header.stamp ||
             !all_finite(
                 {exact.control_speed_mps, exact.resolved_lookahead_distance_m,
                  exact.geometric_steering_tire_angle_rad,
                  exact.curvature_feedforward_steering_rad,
                  exact.raw_steering_tire_angle_rad,
                  exact.requested_output_steering_tire_angle_rad,
                  exact.bounded_steering_tire_angle_rad,
                  exact.requested_steering_tire_rotation_rate_radps,
                  exact.bounded_steering_tire_rotation_rate_radps,
                  exact.limiter_reference_steering_rad, exact.command_dt_sec,
                  exact.wheelbase_m, exact.steering_output_gain,
                  exact.hard_steering_angle_limit_rad,
                  exact.hard_steering_rate_limit_radps,
                  exact.diagnostic_lease_duration_sec}) ||
             !exact.limiter_reference_valid || exact.control_speed_mps < 0.0 ||
             exact.resolved_lookahead_distance_m <= 0.0 ||
             exact.command_dt_sec <= 0.0 || exact.wheelbase_m <= 0.0 ||
             exact.hard_steering_angle_limit_rad <= 0.0 ||
             exact.hard_steering_rate_limit_radps <= 0.0 ||
             exact.diagnostic_lease_duration_sec <= 0.0 ||
             exact.diagnostic_lease_duration_sec > 0.1) {
    witness.exact_snapshot_reason = "exact_scalar_invalid";
  } else {
    const auto first =
        input.applied_trajectory.points.begin() +
        static_cast<std::ptrdiff_t>(input.nearest_trajectory_index);
    const auto last = input.applied_trajectory.points.begin() +
                      static_cast<std::ptrdiff_t>(
                          exact.required_horizon_end_trajectory_index + 1U);
    witness.applied_trajectory.header = input.applied_trajectory.header;
    witness.applied_trajectory.points.assign(first, last);
    witness.exact_snapshot_complete = true;
    witness.exact_snapshot_reason = "complete";
  }
  return envelope;
}

} // namespace simple_pure_pursuit
