#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstdint>
#include <memory>

namespace overtake_transport_contract::c002ay0 {
namespace {

using Authorized =
    multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory;
using Base = multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot;

Digest digest(std::uint8_t value) {
  Digest result{};
  result.fill(value);
  return result;
}

double arcLength(const Authorized::_points_type &points) {
  double result = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double dx =
        points[index].position_x_m - points[index - 1U].position_x_m;
    const double dy =
        points[index].position_y_m - points[index - 1U].position_y_m;
    const double dz =
        points[index].position_z_m - points[index - 1U].position_z_m;
    result += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return result;
}

Authorized proposalFrom(const Base &snapshot) {
  Authorized value;
  value.schema_version = Authorized::SCHEMA_V1_SHADOW;
  value.authority_eligible = false;
  value.plan_stamp = snapshot.record_stamp;
  value.frame_id = snapshot.frame_id;
  value.plan_sample_key.race_arm_epoch = snapshot.race_arm_epoch;
  value.plan_sample_key.planner_instance_id = 0xC002A100U;
  value.plan_sample_key.attempt_id = 1U;
  value.plan_sample_key.target_vehicle_id = "fixed_fixture";
  value.plan_sample_key.pass_direction = 1;
  value.plan_sample_key.connector_transaction_id = 1U;
  value.plan_sample_key.plan_stamp = value.plan_stamp;
  value.plan_sample_key.plan_generation = 1U;
  value.candidate_revision = 1U;
  value.authority_token = 1U;
  value.candidate_type = Authorized::CANDIDATE_PASS_LEFT;
  value.phase = Authorized::PHASE_PASSING;
  value.authorization_state = Authorized::AUTHORIZATION_AUTHORIZED;
  value.source_controller_instance_id = snapshot.controller_instance_id;
  value.source_controller_sequence = snapshot.controller_sequence;
  value.base_lease_id = snapshot.base_lease_id;
  value.base_lease_valid_until = snapshot.lease_valid_until;
  value.base_source_kind = snapshot.base_source_kind;
  value.base_source_stamp = snapshot.base_source_stamp;
  value.base_source_generation = snapshot.base_source_generation;
  value.base_original_point_count = snapshot.base_original_point_count;
  value.base_first_source_index = snapshot.first_source_index;
  value.base_last_source_index = snapshot.last_source_index;
  value.base_nearest_source_index = snapshot.nearest_source_index;
  value.base_source_digest_state = snapshot.base_source_digest_state;
  value.canonical_algorithm_version = snapshot.canonical_algorithm_version;
  value.base_geometry_sha256 = snapshot.base_geometry_sha256;
  value.base_source_sha256 = snapshot.base_source_sha256;
  value.base_snapshot_sha256 = snapshot.snapshot_sha256;
  value.points = snapshot.base_points;
  value.original_candidate_point_count =
      static_cast<std::uint32_t>(value.points.size());
  value.total_arc_length_m = arcLength(value.points);
  value.required_spatial_horizon_m = value.total_arc_length_m;
  value.join_end_arc_length_m = value.total_arc_length_m;
  value.post_join_arc_length_m = 0.0;
  value.safety_snapshot_id = 1U;
  value.safety_evaluation_result = Authorized::SAFETY_PASSED;
  value.safety_evaluation_stamp = value.plan_stamp;
  value.safety_valid_until = snapshot.lease_valid_until;
  value.world_safety_snapshot_sha256 = digest(0x61U);
  value.safety_evaluator_implementation_sha256 = digest(0x71U);
  value.safety_evaluator_config_sha256 = digest(0x81U);
  value.controller_implementation_sha256 =
      snapshot.controller_implementation_sha256;
  value.controller_config_sha256 = snapshot.controller_config_sha256;
  if (!value.points.empty()) {
    const auto &point = value.points.front();
    value.candidate_start_control_pose.position.x = point.position_x_m;
    value.candidate_start_control_pose.position.y = point.position_y_m;
    value.candidate_start_control_pose.position.z = point.position_z_m;
    value.candidate_start_control_pose.orientation.x = point.orientation_x;
    value.candidate_start_control_pose.orientation.y = point.orientation_y;
    value.candidate_start_control_pose.orientation.z = point.orientation_z;
    value.candidate_start_control_pose.orientation.w = point.orientation_w;
  }
  value.candidate_start_control_pose_stamp = value.plan_stamp;

  const auto geometry =
      canonicalizeGeometryV1(value.points, "C002AY0_AUTHORIZED_GEOMETRY_V1");
  if (!geometry.valid()) {
    return Authorized();
  }
  value.geometry_sha256 = geometry.sha256;
  const auto canonical = canonicalizeAuthorizedTrajectoryV1(value);
  if (!canonical.valid()) {
    return Authorized();
  }
  value.candidate_start_control_pose_sha256 = canonical.control_pose_sha256;
  value.safety_proof_sha256 = canonical.safety_proof_sha256;
  const auto complete = canonicalizeAuthorizedTrajectoryV1(value);
  if (!complete.valid()) {
    return Authorized();
  }
  value.payload_sha256 = complete.sha256;
  return value;
}

class ExactProposalRelay : public rclcpp::Node {
public:
  ExactProposalRelay() : Node("c002ay0_exact_proposal_relay") {
    const auto qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    publisher_ = create_publisher<Authorized>(
        "/debug/overtake/state_lattice/authorized_cartesian_trajectory", qos);
    subscription_ = create_subscription<Base>(
        "/control/overtake/base_trajectory_snapshot", qos,
        [this](const Base::SharedPtr snapshot) {
          if (snapshot == nullptr) {
            return;
          }
          auto proposal = proposalFrom(*snapshot);
          if (validateAuthorizedTrajectoryV1(proposal) !=
                  ValidationError::NONE ||
              proposal.authority_eligible) {
            RCLCPP_ERROR(get_logger(),
                         "fixed fixture proposal canonicalization failed");
            return;
          }
          publisher_->publish(proposal);
        });
  }

private:
  rclcpp::Publisher<Authorized>::SharedPtr publisher_;
  rclcpp::Subscription<Base>::SharedPtr subscription_;
};

} // namespace
} // namespace overtake_transport_contract::c002ay0

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<
               overtake_transport_contract::c002ay0::ExactProposalRelay>());
  rclcpp::shutdown();
  return 0;
}
