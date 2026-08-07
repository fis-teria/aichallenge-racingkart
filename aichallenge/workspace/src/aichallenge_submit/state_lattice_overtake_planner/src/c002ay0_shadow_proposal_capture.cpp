#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_capture.hpp"

#include <cmath>
#include <cstring>
#include <time.h>

namespace state_lattice_overtake_planner::c002ay0_shadow {
namespace {

bool finite(double value) noexcept { return std::isfinite(value); }

std::uint64_t monotonicNanoseconds() noexcept {
  timespec stamp{};
  if (clock_gettime(CLOCK_MONOTONIC, &stamp) != 0 || stamp.tv_sec < 0 ||
      stamp.tv_nsec < 0) {
    return 0U;
  }
  return static_cast<std::uint64_t>(stamp.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.tv_nsec);
}

FixedTime toFixedTime(const builtin_interfaces::msg::Time &source) noexcept {
  return {source.sec, source.nanosec};
}

bool copyString(const std::string &source,
                std::array<char, kFixedProposalIdCapacity> *destination,
                std::uint8_t *size) noexcept {
  if (destination == nullptr || size == nullptr || source.empty() ||
      source.size() > destination->size()) {
    return false;
  }
  std::memcpy(destination->data(), source.data(), source.size());
  *size = static_cast<std::uint8_t>(source.size());
  return true;
}

bool copyFrame(
    const std::string &source,
    std::array<char, overtake_transport_contract::c002ay0::kFixedFrameCapacity>
        *destination,
    std::uint16_t *size) noexcept {
  if (destination == nullptr || size == nullptr || source.empty() ||
      source.size() > destination->size()) {
    return false;
  }
  std::memcpy(destination->data(), source.data(), source.size());
  *size = static_cast<std::uint16_t>(source.size());
  return true;
}

} // namespace

bool copyFixedBaseRecord(
    const multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
        &base,
    std::uint64_t session_generation, std::uint64_t session_nonce,
    FixedBaseRecord *record, FixedIncomingBaseProvenance *provenance) noexcept {
  if (record == nullptr || provenance == nullptr || session_generation == 0U ||
      session_nonce == 0U ||
      base.schema_version !=
          multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot::
              SCHEMA_V1_SHADOW ||
      base.authority_eligible ||
      overtake_transport_contract::c002ay0::validateBaseSnapshotV1(base) !=
          overtake_transport_contract::c002ay0::ValidationError::NONE ||
      base.base_points.size() < 2U ||
      base.base_points.size() > record->points.size() ||
      base.first_source_index != 0U ||
      base.last_source_index + 1U != base.base_points.size() ||
      base.nearest_source_index >= base.base_points.size() ||
      base.base_original_point_count != base.base_points.size() ||
      base.race_arm_epoch == 0U || base.controller_instance_id == 0U ||
      base.controller_sequence == 0U || base.base_lease_id == 0U ||
      base.base_source_kind == 0U || base.base_source_generation == 0U ||
      !copyFrame(base.frame_id, &record->frame, &record->frame_size)) {
    return false;
  }
  *record = FixedBaseRecord{};
  record->session_generation = session_generation;
  record->session_nonce = session_nonce;
  record->record_stamp = toFixedTime(base.record_stamp);
  if (!copyFrame(base.frame_id, &record->frame, &record->frame_size)) {
    return false;
  }
  record->race_arm_epoch = base.race_arm_epoch;
  record->controller_instance_id = base.controller_instance_id;
  record->controller_sequence = base.controller_sequence;
  record->base_lease_id = base.base_lease_id;
  record->lease_valid_until = toFixedTime(base.lease_valid_until);
  record->base_source_kind = base.base_source_kind;
  record->base_source_stamp = toFixedTime(base.base_source_stamp);
  record->base_source_generation = base.base_source_generation;
  record->base_original_point_count = base.base_original_point_count;
  record->nearest_source_index = base.nearest_source_index;
  record->point_count = static_cast<std::uint32_t>(base.base_points.size());
  for (std::size_t index = 0U; index < base.base_points.size(); ++index) {
    const auto &source = base.base_points[index];
    auto &destination = record->points[index];
    destination.time_from_start = {source.time_from_start.sec,
                                   source.time_from_start.nanosec};
    destination.position_x_m = source.position_x_m;
    destination.position_y_m = source.position_y_m;
    destination.position_z_m = source.position_z_m;
    destination.orientation_x = source.orientation_x;
    destination.orientation_y = source.orientation_y;
    destination.orientation_z = source.orientation_z;
    destination.orientation_w = source.orientation_w;
    destination.longitudinal_velocity_mps = source.longitudinal_velocity_mps;
    destination.lateral_velocity_mps = source.lateral_velocity_mps;
    destination.acceleration_mps2 = source.acceleration_mps2;
    destination.heading_rate_rps = source.heading_rate_rps;
    destination.front_wheel_angle_rad = source.front_wheel_angle_rad;
    destination.rear_wheel_angle_rad = source.rear_wheel_angle_rad;
  }
  record->controller_implementation_sha256 =
      base.controller_implementation_sha256;
  record->controller_config_sha256 = base.controller_config_sha256;
  *provenance = {base.base_source_sha256, base.base_geometry_sha256,
                 base.snapshot_sha256};
  return true;
}

FixedProposalCaptureResult buildFixedProposalRecord(
    const FixedBaseRecord &base, const FixedIncomingBaseProvenance &provenance,
    const CandidateTrajectory &candidate, const EgoState &ego,
    const std::vector<OpponentState> &opponents,
    const Ay0ShadowSafetyEvidence &evidence,
    const builtin_interfaces::msg::Time &plan_stamp,
    const Ay0ShadowProposalIdentity &identity, std::uint64_t session_generation,
    std::uint64_t session_nonce, FixedProposalRecord *record) noexcept {
  if (record == nullptr || session_generation == 0U || session_nonce == 0U ||
      base.point_count < 2U || base.point_count > base.points.size() ||
      candidate.dense.size() < 2U ||
      candidate.dense.size() >
          overtake_transport_contract::c002ay0::kMaxCartesianPoints ||
      !candidate.feasible || !ego.valid || !ego.frenet.valid ||
      opponents.size() > kFixedProposalOpponentCapacity) {
    return FixedProposalCaptureResult::kCandidateInvalid;
  }
  if (identity.planner_instance_id == 0U || identity.attempt_id == 0U ||
      identity.connector_transaction_id == 0U ||
      identity.authority_token == 0U || identity.safety_snapshot_id == 0U ||
      identity.plan_generation == 0U || identity.candidate_revision == 0U ||
      identity.target_id.empty()) {
    return FixedProposalCaptureResult::kIdentityInvalid;
  }
  *record = FixedProposalRecord{};
  record->capture_monotonic_ns = monotonicNanoseconds();
  if (record->capture_monotonic_ns == 0U) {
    return FixedProposalCaptureResult::kCandidateInvalid;
  }
  record->session_generation = session_generation;
  record->session_nonce = session_nonce;
  record->planner_instance_id = identity.planner_instance_id;
  record->attempt_id = identity.attempt_id;
  record->connector_transaction_id = identity.connector_transaction_id;
  record->authority_token = identity.authority_token;
  record->safety_snapshot_id = identity.safety_snapshot_id;
  record->plan_generation = identity.plan_generation;
  record->candidate_revision = identity.candidate_revision;
  record->plan_stamp = toFixedTime(plan_stamp);
  record->evaluator_implementation_sha256 =
      evidence.evaluator_implementation_sha256;
  record->evaluator_config_sha256 = evidence.evaluator_config_sha256;
  if (!copyString(identity.target_id, &record->target_id,
                  &record->target_id_size)) {
    return FixedProposalCaptureResult::kIdentityInvalid;
  }
  record->candidate_feasible = 1U;
  record->candidate_goal_d_m = candidate.goal_d_m;
  record->candidate_point_count =
      static_cast<std::uint16_t>(candidate.dense.size());
  double previous_time = -1.0;
  for (std::size_t index = 0U; index < candidate.dense.size(); ++index) {
    const auto &source = candidate.dense[index];
    if (!finite(source.x) || !finite(source.y) || !finite(source.yaw) ||
        !finite(source.s) || !finite(source.d) || !finite(source.kappa) ||
        !finite(source.speed_mps) || !finite(source.time_sec) ||
        source.speed_mps < 0.0 || source.time_sec < 0.0 ||
        source.time_sec <= previous_time) {
      return FixedProposalCaptureResult::kCandidateInvalid;
    }
    record->candidate_points[index] = {
        source.x, source.y,     source.yaw,       source.s,
        source.d, source.kappa, source.speed_mps, source.time_sec};
    previous_time = source.time_sec;
  }
  for (double value :
       {ego.x, ego.y, ego.yaw, ego.stamp_sec, ego.speed_mps, ego.yaw_rate_radps,
        ego.curvature, ego.frenet.s, ego.frenet.d, ego.frenet.yaw_error}) {
    if (!finite(value)) {
      return FixedProposalCaptureResult::kCandidateInvalid;
    }
  }
  record->ego = {ego.x,
                 ego.y,
                 ego.yaw,
                 ego.stamp_sec,
                 ego.speed_mps,
                 ego.yaw_rate_radps,
                 ego.curvature,
                 ego.frenet.s,
                 ego.frenet.d,
                 ego.frenet.yaw_error,
                 static_cast<std::uint32_t>(ego.frenet.segment_index),
                 1U,
                 1U};
  record->opponent_count = static_cast<std::uint8_t>(opponents.size());
  for (std::size_t index = 0U; index < opponents.size(); ++index) {
    const auto &source = opponents[index];
    auto &destination = record->opponents[index];
    if (!source.valid || !source.frenet.valid ||
        !copyString(source.id, &destination.id, &destination.id_size)) {
      return FixedProposalCaptureResult::kOpponentLimit;
    }
    for (double value :
         {source.x, source.y, source.yaw, source.stamp_sec, source.speed_mps,
          source.vx_mps, source.vy_mps, source.sigma_x_m, source.sigma_y_m,
          source.uncertainty_x_m, source.uncertainty_y_m, source.frenet.s,
          source.frenet.d, source.frenet.yaw_error}) {
      if (!finite(value)) {
        return FixedProposalCaptureResult::kOpponentLimit;
      }
    }
    destination.x_m = source.x;
    destination.y_m = source.y;
    destination.yaw_rad = source.yaw;
    destination.stamp_sec = source.stamp_sec;
    destination.speed_mps = source.speed_mps;
    destination.vx_mps = source.vx_mps;
    destination.vy_mps = source.vy_mps;
    destination.sigma_x_m = source.sigma_x_m;
    destination.sigma_y_m = source.sigma_y_m;
    destination.uncertainty_x_m = source.uncertainty_x_m;
    destination.uncertainty_y_m = source.uncertainty_y_m;
    destination.frenet_s_m = source.frenet.s;
    destination.frenet_d_m = source.frenet.d;
    destination.frenet_yaw_error_rad = source.frenet.yaw_error;
    destination.frenet_segment_index =
        static_cast<std::uint32_t>(source.frenet.segment_index);
    destination.valid = 1U;
    destination.frenet_valid = 1U;
  }
  record->base = base;
  record->incoming_base = provenance;
  return FixedProposalCaptureResult::kBuilt;
}

} // namespace state_lattice_overtake_planner::c002ay0_shadow
