#pragma once

// Fixed test-only snapshot from:
// output/20260802-safegate2-object-owner-01/d1/rosbag2_autoware
// Planner metrics generation 602 and the matching source-stamped V2X sample.
// This fixture is evidence input only. It must not be linked into the ROS node.
namespace state_lattice_overtake_planner::test_fixture
{

struct Gate2Generation602
{
  static constexpr double ego_x_m = 89630.63521527934;
  static constexpr double ego_y_m = 43132.74313035868;
  static constexpr double ego_yaw_rad = 2.2711862046600273;
  static constexpr double ego_speed_mps = 0.8249723123782591;
  static constexpr double ego_yaw_rate_radps = 0.06738118458035311;
  static constexpr double ego_source_stamp_sec = 52.324998830;
  static constexpr double opponent_source_stamp_sec = 52.249998832;

  static constexpr double d2_x_m = 89628.9140625;
  static constexpr double d2_y_m = 43131.0;
  static constexpr double d3_x_m = 89624.7109375;
  static constexpr double d3_y_m = 43137.74609375;
  static constexpr double opponent_covariance_m2 = 0.004999999888241291;

  static constexpr double continuation_goal_d_m = -2.4;
  static constexpr double continuation_arc_m = 9.787;
  static constexpr double continuation_tangent_scale = 1.0;
  static constexpr int observed_total_cost = 355;
  static constexpr int observed_reference_cost = 200;
  static constexpr int observed_wall_cost = 0;
  static constexpr int observed_object_cost = 158;
};

// First record in bag order satisfying the pre-declared secondary eligibility
// predicate: D2 continuation active, D3 early-aware active with two distinct
// fresh stamps, and D3 classified as an accepted forward target.
struct Gate2FirstSecondaryEligible
{
  static constexpr double ego_x_m = 89630.68752067657;
  static constexpr double ego_y_m = 43132.67965431801;
  static constexpr double ego_yaw_rad = 2.2646599856237195;
  static constexpr double ego_speed_mps = 0.8293744192828102;
  static constexpr double ego_yaw_rate_radps = 0.06854114341098844;
  static constexpr double ego_source_stamp_sec = 52.224998832;
  static constexpr double opponent_source_stamp_sec = 52.199998833;

  static constexpr double d2_x_m = 89628.9140625;
  static constexpr double d2_y_m = 43131.0;
  static constexpr double d3_x_m = 89624.7109375;
  static constexpr double d3_y_m = 43137.74609375;
  static constexpr double opponent_covariance_m2 = 0.004999999888241291;

  static constexpr double continuation_goal_d_m = -2.4;
  static constexpr double continuation_arc_m = 9.807;
  static constexpr double continuation_tangent_scale = 1.0;
  static constexpr int observed_total_cost = 355;
  static constexpr int observed_reference_cost = 200;
  static constexpr int observed_wall_cost = 0;
  static constexpr int observed_object_cost = 158;
};

}  // namespace state_lattice_overtake_planner::test_fixture
