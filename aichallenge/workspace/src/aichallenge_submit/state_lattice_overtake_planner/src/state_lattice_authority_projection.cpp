#include "state_lattice_overtake_planner/state_lattice_authority_projection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "overtake_transport_contract/c002ay0_canonical.hpp"

namespace state_lattice_overtake_planner {
namespace {

using Authorized =
    multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory;
using OvertakePlan = multi_purpose_mpc_ros_msgs::msg::OvertakePlan;

bool digestPresent(const std::array<std::uint8_t, 32U> &digest) {
  return std::any_of(digest.cbegin(), digest.cend(),
                     [](std::uint8_t value) { return value != 0U; });
}

} // namespace

const StateLatticeAuthorityTransaction *
StateLatticeAuthorityTransactionState::select(const std::string &target_id,
                                               std::int8_t pass_direction) {
  if (target_id.empty() || (pass_direction != -1 && pass_direction != 1)) {
    active_.reset();
    return nullptr;
  }
  if (active_.has_value() && active_->target_id == target_id &&
      active_->pass_direction == pass_direction) {
    return &active_.value();
  }
  if (last_attempt_id_ == std::numeric_limits<std::uint32_t>::max()) {
    active_.reset();
    return nullptr;
  }
  ++last_attempt_id_;
  StateLatticeAuthorityTransaction transaction;
  transaction.attempt_id = last_attempt_id_;
  transaction.connector_transaction_id = last_attempt_id_;
  transaction.authority_token = last_attempt_id_;
  transaction.target_id = target_id;
  transaction.pass_direction = pass_direction;
  active_ = std::move(transaction);
  return &active_.value();
}

StateLatticeAuthorityProjection projectStateLatticeAuthority(
    const Authorized &proposal,
    const multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus
        *tracking_status,
    const Authorized *predecessor_proposal,
    bool predecessor_fresh, bool race_armed, std::uint64_t race_arm_epoch) {
  StateLatticeAuthorityProjection result;
  const auto &key = proposal.plan_sample_key;
  const bool pass_candidate =
      proposal.candidate_type == Authorized::CANDIDATE_PASS_LEFT ||
      proposal.candidate_type == Authorized::CANDIDATE_PASS_RIGHT;
  if (!race_armed || race_arm_epoch == 0U ||
      key.race_arm_epoch != race_arm_epoch) {
    result.reason = "race_authority";
    return result;
  }
  if (overtake_transport_contract::c002ay0::validateAuthorizedTrajectoryV1(
          proposal) !=
          overtake_transport_contract::c002ay0::ValidationError::NONE ||
      !pass_candidate || proposal.phase != Authorized::PHASE_PASSING ||
      proposal.authorization_state != Authorized::AUTHORIZATION_AUTHORIZED ||
      proposal.safety_evaluation_result != Authorized::SAFETY_PASSED ||
      key.plan_generation == 0U || key.planner_instance_id == 0U ||
      key.attempt_id == 0U || key.target_vehicle_id.empty() ||
      key.attempt_id > std::numeric_limits<std::uint32_t>::max() ||
      (key.pass_direction != -1 && key.pass_direction != 1) ||
      key.connector_transaction_id == 0U ||
      proposal.candidate_revision == 0U || proposal.authority_token == 0U ||
      proposal.points.size() < 2U || !digestPresent(proposal.geometry_sha256) ||
      !std::isfinite(proposal.points.front().longitudinal_velocity_mps) ||
      proposal.points.front().longitudinal_velocity_mps <= 0.0F) {
    result.reason = "proposal_invalid";
    return result;
  }

  // State owns candidate safety and the entry-speed cap.  It does not wait for
  // PP feedback before publishing that authorization: Mux independently
  // requires a fresh, exact current-generation bounded PP envelope before it
  // can grant motion.  This avoids a planner-generation feedback race while
  // preserving the execution-side identity and freshness checks.
  (void)tracking_status;
  (void)predecessor_proposal;
  (void)predecessor_fresh;
  const bool motion_ready = true;

  // A fresh N-1 readiness proof may release only the newly safety-evaluated N
  // candidate. Mux still independently requires an exact usable N PP command,
  // so predecessor readiness cannot authorize execution of untracked geometry.
  const Authorized &authority_proposal = proposal;
  const auto &authority_key = authority_proposal.plan_sample_key;

  auto &plan = result.plan;
  plan.header.stamp = authority_proposal.plan_stamp;
  plan.header.frame_id = authority_proposal.frame_id;
  plan.phase = OvertakePlan::PASSING;
  plan.plan_generation = authority_key.plan_generation;
  plan.attempt_id = static_cast<std::uint32_t>(authority_key.attempt_id);
  plan.target_vehicle_id = authority_key.target_vehicle_id;
  plan.pass_direction = authority_key.pass_direction;
  plan.trajectory_authorized = true;
  plan.lateral_maneuver_required = true;
  plan.decision_reason = motion_ready ? "state_lattice_ready"
                                      : "state_lattice_pass_warmup";
  plan.authorization_failure_mask = 0U;
  plan.safety_inputs_complete = true;
  plan.tracking_usable = motion_ready;
  plan.trajectory_publishable = true;
  plan.constraint_reason =
      motion_ready ? "state_lattice_release_authorized"
                   : "release_pending_safe_cycles";
  plan.lateral_stop_authority_kind =
      motion_ready ? OvertakePlan::LATERAL_STOP_NONE
                   : OvertakePlan::LATERAL_STOP_PASS_WARMUP;
  plan.lateral_stop_transaction_pass_direction =
      motion_ready ? 0 : authority_key.pass_direction;
  plan.lateral_stop_authority_token =
      motion_ready ? 0U : authority_proposal.authority_token;
  plan.aw2_identity_schema_version = 1U;
  plan.planner_instance_id = authority_key.planner_instance_id;
  plan.race_arm_epoch = authority_key.race_arm_epoch;
  plan.connector_transaction_id = authority_key.connector_transaction_id;
  plan.candidate_revision = authority_proposal.candidate_revision;
  plan.candidate_content_sha256 = authority_proposal.geometry_sha256;
  plan.trajectory.header = plan.header;
  plan.trajectory.points.reserve(authority_proposal.points.size());
  for (const auto &source : authority_proposal.points) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;
    point.time_from_start = source.time_from_start;
    point.pose.position.x = source.position_x_m;
    point.pose.position.y = source.position_y_m;
    point.pose.position.z = source.position_z_m;
    point.pose.orientation.x = source.orientation_x;
    point.pose.orientation.y = source.orientation_y;
    point.pose.orientation.z = source.orientation_z;
    point.pose.orientation.w = source.orientation_w;
    point.longitudinal_velocity_mps = source.longitudinal_velocity_mps;
    point.lateral_velocity_mps = source.lateral_velocity_mps;
    point.acceleration_mps2 = source.acceleration_mps2;
    point.heading_rate_rps = source.heading_rate_rps;
    point.front_wheel_angle_rad = source.front_wheel_angle_rad;
    point.rear_wheel_angle_rad = source.rear_wheel_angle_rad;
    plan.trajectory.points.push_back(std::move(point));
  }

  auto &constraint = result.constraint;
  constraint.header = plan.header;
  constraint.constraint_generation = authority_key.plan_generation;
  constraint.plan_generation = authority_key.plan_generation;
  constraint.valid = true;
  constraint.stop_requested = !motion_ready;
  constraint.release_authorized = motion_ready;
  // A STOP constraint still carries the positive speed cap of the authorized
  // candidate. stop_requested is the independent longitudinal hold signal;
  // encoding STOP as a zero cap would violate the existing Mux contract and
  // make the otherwise valid authority bundle unusable.
  constraint.speed_limit_mps =
      authority_proposal.points.front().longitudinal_velocity_mps;
  constraint.required_brake_decel_mps2 = 0.0F;
  constraint.reason = plan.constraint_reason;

  result.valid = true;
  result.warmup = !motion_ready;
  result.reason = motion_ready ? "ready" : "pass_warmup";
  return result;
}

} // namespace state_lattice_overtake_planner
