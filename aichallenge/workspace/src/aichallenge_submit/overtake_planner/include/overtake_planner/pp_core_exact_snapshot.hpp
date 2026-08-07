#pragma once

#include "overtake_planner/cartesian_trackability_evaluator.hpp"

#include <array>
#include <cstdint>
#include <multi_purpose_mpc_ros_msgs/msg/controller_execution_envelope.hpp>
#include <string>

namespace overtake_planner {

struct PurePursuitCandidateBinding {
  std::uint64_t race_arm_epoch{0U};
  std::uint64_t planner_instance_id{0U};
  std::uint64_t attempt_id{0U};
  std::string target_vehicle_id{};
  std::int8_t pass_direction{0};
  std::uint64_t connector_transaction_id{0U};
  double plan_stamp_sec{0.0};
  std::uint32_t plan_generation{0U};
  std::uint32_t candidate_revision{0U};
  std::array<std::uint8_t, 32U> candidate_content_sha256{};
};

struct PurePursuitExactSnapshot {
  bool valid{false};
  std::string reason{"not_evaluated"};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t command_sequence{0U};
  double command_stamp_sec{0.0};
  double valid_until_sec{0.0};
  std::array<std::uint8_t, 32U> bounded_geometry_sha256{};
  PurePursuitCandidateBinding binding{};
  CartesianTrackabilityInput evaluator_input{};
  std::size_t selected_lookahead_source_index{0U};
  bool lookahead_endpoint_fallback{false};
  double geometric_steering_tire_angle_rad{0.0};
  double raw_steering_tire_angle_rad{0.0};
  double requested_output_steering_tire_angle_rad{0.0};
  double bounded_steering_tire_angle_rad{0.0};
  double requested_steering_rate_radps{0.0};
  double bounded_steering_rate_radps{0.0};
  bool steering_angle_limited{false};
  bool steering_rate_limited{false};
};

PurePursuitExactSnapshot validatePurePursuitExactSnapshotV2(
    const multi_purpose_mpc_ros_msgs::msg::ControllerExecutionEnvelope
        &envelope,
    const PurePursuitCandidateBinding &expected, double receive_time_sec,
    double future_tolerance_sec = 0.05);

} // namespace overtake_planner
